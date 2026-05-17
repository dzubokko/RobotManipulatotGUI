from datetime import datetime
import math
import os
import signal
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, Tuple

from PyQt6.QtCore import QProcess, QTimer
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


Vector3 = Tuple[float, float, float]


class ManualControlUI(QWidget):
    """
    Вкладка ручного управления роботом.

    Важное изменение:
    ручное движение теперь сглажено. GUI не отправляет короткую
    траекторию каждые 50 мс. Вместо этого используется:
    - плавный разгон;
    - плавное торможение;
    - ограничение частоты команд;
    - более длинная траектория для ros2_control/Gazebo.
    """

    COLOR_BG = "#171717"
    COLOR_PANEL = "#202020"
    COLOR_PANEL_2 = "#252525"
    COLOR_BORDER = "#3a3a3a"
    COLOR_TEXT = "#f2f2f2"
    COLOR_MUTED = "#a9a9a9"
    COLOR_ACCENT = "#0a84ff"
    COLOR_ACCENT_HOVER = "#1e90ff"
    COLOR_DANGER = "#d83b3b"
    COLOR_DANGER_HOVER = "#e94b4b"
    COLOR_WARNING = "#f59f00"
    COLOR_LOG = "#101010"

    def __init__(self, robot_bridge):
        super().__init__()

        self.robot = robot_bridge

        self.gazebo_process = None
        self.gazebo_pid = None
        self.gazebo_master_uri = None

        # Скорости ручного управления.
        # linear_speed_mps — реальная скорость TCP в м/с.
        # angular_speed_rad_s — скорость поворота TCP в рад/с.
        self.speed_levels = [
            {
                "label": "10 мм/с",
                "scale": 0.5,
                "linear_speed_mps": 0.010,
                "angular_speed_rad_s": math.radians(5.0),
            },
            {
                "label": "50 мм/с",
                "scale": 2.5,
                "linear_speed_mps": 0.050,
                "angular_speed_rad_s": math.radians(12.0),
            },
            {
                "label": "100 мм/с",
                "scale": 5.0,
                "linear_speed_mps": 0.100,
                "angular_speed_rad_s": math.radians(20.0),
            },
        ]
        self.speed_idx = 1

        # Целевые направления от кнопок.
        self.current_move: Vector3 = (0.0, 0.0, 0.0)
        self.current_drot: Vector3 = (0.0, 0.0, 0.0)

        # Сглаженные направления, которые реально отправляются роботу.
        self.smooth_move: Vector3 = (0.0, 0.0, 0.0)
        self.smooth_drot: Vector3 = (0.0, 0.0, 0.0)

        # Параметры сглаживания.
        self.manual_timer_interval_ms = 40
        self.manual_send_period_sec = 0.14
        self.manual_trajectory_duration_sec = 0.40
        self.manual_ramp_time_sec = 0.30

        self._last_manual_tick_time = time.monotonic()
        self._last_manual_send_time = 0.0
        self._last_manual_error_log_time = 0.0

        self.last_motion_state = None
        self.last_estop_state = None
        self.last_joint_states_state = None
        self.last_motion_log_time = 0.0
        self.last_gazebo_log_time = 0.0

        self._pending_log_lines = []
        self._dropped_log_lines = 0
        self._max_pending_log_lines = 200

        self._ros_data_lock = threading.Lock()
        self._latest_ros_data = None
        self._has_pending_ros_data = False

        self.logs_visible = True

        self.manual_timer = QTimer(self)
        self.manual_timer.setInterval(self.manual_timer_interval_ms)
        self.manual_timer.timeout.connect(self.on_manual_timer)

        self.log_flush_timer = QTimer(self)
        self.log_flush_timer.setInterval(250)
        self.log_flush_timer.timeout.connect(self.flush_log)

        self.ros_update_timer = QTimer(self)
        self.ros_update_timer.setInterval(100)
        self.ros_update_timer.timeout.connect(self.process_pending_ros_update)

        self.init_ui()
        self.update_speed()

        self.log_flush_timer.start()
        self.ros_update_timer.start()

        if self.robot is not None:
            self.robot.register_callback(self.enqueue_ros_update)

        app = QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self.stop_gazebo_process)

        self.append_log("GUI ручного управления запущен.")
        self.append_log("Режим ручного движения: сглаженный streaming-control.")
        self.append_log("Основной запуск проекта: python3 -m robot_arm_controller.app")

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def init_ui(self):
        self.setStyleSheet(self.base_stylesheet())

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(12, 10, 12, 10)
        main_layout.setSpacing(10)

        header_layout = QHBoxLayout()

        title = QLabel("РУЧНОЕ УПРАВЛЕНИЕ")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setStyleSheet(f"color: {self.COLOR_TEXT};")

        header_layout.addWidget(title)
        header_layout.addStretch()

        self.toggle_logs_btn = QPushButton("Скрыть логи")
        self.toggle_logs_btn.setStyleSheet(self.button_style("secondary"))
        self.toggle_logs_btn.clicked.connect(self.toggle_logs)

        header_layout.addWidget(self.toggle_logs_btn)
        main_layout.addLayout(header_layout)

        content = QHBoxLayout()
        content.setSpacing(10)

        left_panel = self.build_motion_panel()
        center_panel = self.build_status_panel()
        self.log_panel = self.build_log_panel()

        content.addWidget(left_panel, 1)
        content.addWidget(center_panel, 1)
        content.addWidget(self.log_panel, 1)

        main_layout.addLayout(content, 1)
        self.setLayout(main_layout)

    def build_motion_panel(self) -> QGroupBox:
        group = QGroupBox("ДВИЖЕНИЕ И БЕЗОПАСНОСТЬ")
        layout = QVBoxLayout(group)
        layout.setSpacing(10)

        linear_group = QGroupBox("Линейное движение TCP")
        linear_layout = QGridLayout(linear_group)
        linear_layout.setSpacing(8)

        z_plus = self.make_button("Z+", "primary", 90, 56)
        z_plus.pressed.connect(lambda: self.start_move(0.0, 0.0, +1.0))
        z_plus.released.connect(self.stop_move)
        linear_layout.addWidget(z_plus, 0, 1)

        z_minus = self.make_button("Z-", "primary", 90, 56)
        z_minus.pressed.connect(lambda: self.start_move(0.0, 0.0, -1.0))
        z_minus.released.connect(self.stop_move)
        linear_layout.addWidget(z_minus, 2, 1)

        y_minus = self.make_button("Y-", "primary", 90, 56)
        y_minus.pressed.connect(lambda: self.start_move(0.0, -1.0, 0.0))
        y_minus.released.connect(self.stop_move)
        linear_layout.addWidget(y_minus, 1, 0)

        y_plus = self.make_button("Y+", "primary", 90, 56)
        y_plus.pressed.connect(lambda: self.start_move(0.0, +1.0, 0.0))
        y_plus.released.connect(self.stop_move)
        linear_layout.addWidget(y_plus, 1, 2)

        x_plus = self.make_button("X+", "primary", 90, 56)
        x_plus.pressed.connect(lambda: self.start_move(+1.0, 0.0, 0.0))
        x_plus.released.connect(self.stop_move)
        linear_layout.addWidget(x_plus, 3, 1)

        x_minus = self.make_button("X-", "primary", 90, 56)
        x_minus.pressed.connect(lambda: self.start_move(-1.0, 0.0, 0.0))
        x_minus.released.connect(self.stop_move)
        linear_layout.addWidget(x_minus, 4, 1)

        layout.addWidget(linear_group)

        angular_group = QGroupBox("Поворот TCP")
        angular_layout = QGridLayout(angular_group)
        angular_layout.setSpacing(8)

        rx_minus = self.make_button("RX-", "secondary", 90, 48)
        rx_minus.pressed.connect(lambda: self.start_rotate(+1.0, 0.0, 0.0))
        rx_minus.released.connect(self.stop_rotate)
        angular_layout.addWidget(rx_minus, 0, 0)

        rx_plus = self.make_button("RX+", "secondary", 90, 48)
        rx_plus.pressed.connect(lambda: self.start_rotate(-1.0, 0.0, 0.0))
        rx_plus.released.connect(self.stop_rotate)
        angular_layout.addWidget(rx_plus, 0, 1)

        ry_minus = self.make_button("RY-", "secondary", 90, 48)
        ry_minus.pressed.connect(lambda: self.start_rotate(0.0, +1.0, 0.0))
        ry_minus.released.connect(self.stop_rotate)
        angular_layout.addWidget(ry_minus, 1, 0)

        ry_plus = self.make_button("RY+", "secondary", 90, 48)
        ry_plus.pressed.connect(lambda: self.start_rotate(0.0, -1.0, 0.0))
        ry_plus.released.connect(self.stop_rotate)
        angular_layout.addWidget(ry_plus, 1, 1)

        rz_minus = self.make_button("RZ-", "secondary", 90, 48)
        rz_minus.pressed.connect(lambda: self.start_rotate(0.0, 0.0, +1.0))
        rz_minus.released.connect(self.stop_rotate)
        angular_layout.addWidget(rz_minus, 2, 0)

        rz_plus = self.make_button("RZ+", "secondary", 90, 48)
        rz_plus.pressed.connect(lambda: self.start_rotate(0.0, 0.0, -1.0))
        rz_plus.released.connect(self.stop_rotate)
        angular_layout.addWidget(rz_plus, 2, 1)

        layout.addWidget(angular_group)

        safety_group = QGroupBox("Безопасность")
        safety_layout = QVBoxLayout(safety_group)
        safety_layout.setSpacing(8)

        stop_btn = self.make_button("STOP MOTION", "warning")
        stop_btn.clicked.connect(self.on_stop_motion)
        safety_layout.addWidget(stop_btn)

        estop_btn = self.make_button("EMERGENCY STOP", "danger")
        estop_btn.clicked.connect(self.on_emergency_stop)
        safety_layout.addWidget(estop_btn)

        reset_estop_btn = self.make_button("RESET E-STOP", "primary")
        reset_estop_btn.clicked.connect(self.on_reset_emergency_stop)
        safety_layout.addWidget(reset_estop_btn)

        layout.addWidget(safety_group)
        layout.addStretch()

        return group

    def build_status_panel(self) -> QGroupBox:
        group = QGroupBox("СТАТУС И УПРАВЛЕНИЕ")
        layout = QVBoxLayout(group)
        layout.setSpacing(10)

        self.system_status_display = QLabel(
            "ROS: ---\n"
            "Joint states: ---\n"
            "Motion: ---\n"
            "E-STOP: ---"
        )
        self.system_status_display.setStyleSheet(self.info_box_style())

        layout.addWidget(self.section_title("Системный статус"))
        layout.addWidget(self.system_status_display)

        self.position_display = QLabel(
            "J1: 0.0° J2: 0.0° J3: 0.0°\n"
            "J4: 0.0° J5: 0.0° J6: 0.0°"
        )
        self.position_display.setStyleSheet(self.info_box_style(monospace=True))

        layout.addWidget(self.section_title("Углы суставов"))
        layout.addWidget(self.position_display)

        self.pose_display = QLabel(
            "X: --- mm Y: --- mm Z: --- mm\n"
            "RX: ---° RY: ---° RZ: ---°"
        )
        self.pose_display.setStyleSheet(self.info_box_style(monospace=True))

        layout.addWidget(self.section_title("Координаты TCP"))
        layout.addWidget(self.pose_display)

        self.gripper_display = QLabel("Захват: --- mm")
        self.gripper_display.setStyleSheet(self.info_box_style(monospace=True))

        layout.addWidget(self.section_title("Захват"))
        layout.addWidget(self.gripper_display)

        speed_group = QGroupBox("Скорость подачи")
        speed_layout = QVBoxLayout(speed_group)

        self.speed_combo = QComboBox()
        for level in self.speed_levels:
            self.speed_combo.addItem(level["label"])

        self.speed_combo.setCurrentIndex(self.speed_idx)
        self.speed_combo.currentIndexChanged.connect(self.on_speed_changed)
        self.speed_combo.setStyleSheet(self.combo_style())

        speed_layout.addWidget(self.speed_combo)
        layout.addWidget(speed_group)

        home_btn = self.make_button("ИСХОДНАЯ ПОЗИЦИЯ", "primary")
        home_btn.clicked.connect(self.on_home)
        layout.addWidget(home_btn)

        gripper_group = QGroupBox("Управление захватом")
        gripper_layout = QHBoxLayout(gripper_group)

        open_gripper_btn = self.make_button("ОТКРЫТЬ", "secondary")
        open_gripper_btn.clicked.connect(self.on_gripper_open)
        gripper_layout.addWidget(open_gripper_btn)

        close_gripper_btn = self.make_button("ЗАКРЫТЬ", "secondary")
        close_gripper_btn.clicked.connect(self.on_gripper_close)
        gripper_layout.addWidget(close_gripper_btn)

        layout.addWidget(gripper_group)

        simulation_group = QGroupBox("Симуляция")
        simulation_layout = QVBoxLayout(simulation_group)

        start_sim_btn = self.make_button("ОТКРЫТЬ СИМУЛЯЦИЮ", "primary")
        start_sim_btn.clicked.connect(self.on_start_gazebo)
        simulation_layout.addWidget(start_sim_btn)

        stop_sim_btn = self.make_button("ОСТАНОВИТЬ СИМУЛЯЦИЮ", "secondary")
        stop_sim_btn.clicked.connect(self.on_stop_gazebo)
        simulation_layout.addWidget(stop_sim_btn)

        layout.addWidget(simulation_group)
        layout.addStretch()

        return group

    def build_log_panel(self) -> QGroupBox:
        group = QGroupBox("ЖУРНАЛ СОБЫТИЙ")
        layout = QVBoxLayout(group)
        layout.setSpacing(8)

        self.log_display = QPlainTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setMaximumBlockCount(200)
        self.log_display.setStyleSheet(
            f"""
            QPlainTextEdit {{
                background-color: {self.COLOR_LOG};
                color: #d7ffd7;
                border: 1px solid {self.COLOR_BORDER};
                border-radius: 8px;
                padding: 8px;
                font-family: Courier;
                font-size: 10px;
            }}
            """
        )

        layout.addWidget(self.log_display)

        clear_log_btn = self.make_button("ОЧИСТИТЬ ЛОГ", "secondary")
        clear_log_btn.clicked.connect(self.clear_log)
        layout.addWidget(clear_log_btn)

        return group

    # ------------------------------------------------------------------
    # Styles
    # ------------------------------------------------------------------

    def base_stylesheet(self) -> str:
        return f"""
        QWidget {{
            background-color: {self.COLOR_BG};
            color: {self.COLOR_TEXT};
            font-size: 11px;
        }}

        QGroupBox {{
            background-color: {self.COLOR_PANEL};
            color: {self.COLOR_TEXT};
            border: 1px solid {self.COLOR_BORDER};
            border-radius: 10px;
            margin-top: 10px;
            padding: 10px;
            font-weight: bold;
        }}

        QGroupBox::title {{
            subcontrol-origin: margin;
            left: 12px;
            padding: 0 5px;
            color: {self.COLOR_MUTED};
        }}

        QLabel {{
            color: {self.COLOR_TEXT};
        }}
        """

    def button_style(self, variant: str = "primary") -> str:
        if variant == "danger":
            bg = self.COLOR_DANGER
            hover = self.COLOR_DANGER_HOVER
            border = self.COLOR_DANGER
        elif variant == "warning":
            bg = self.COLOR_PANEL_2
            hover = "#303030"
            border = self.COLOR_WARNING
        elif variant == "secondary":
            bg = self.COLOR_PANEL_2
            hover = "#303030"
            border = self.COLOR_BORDER
        else:
            bg = self.COLOR_ACCENT
            hover = self.COLOR_ACCENT_HOVER
            border = self.COLOR_ACCENT

        return f"""
        QPushButton {{
            background-color: {bg};
            color: {self.COLOR_TEXT};
            border: 1px solid {border};
            border-radius: 8px;
            font-weight: bold;
            font-size: 10px;
            padding: 8px 12px;
        }}

        QPushButton:hover {{
            background-color: {hover};
        }}

        QPushButton:pressed {{
            background-color: #0f0f0f;
        }}

        QPushButton:disabled {{
            background-color: #333333;
            color: #777777;
            border: 1px solid #333333;
        }}
        """

    def make_button(
        self,
        text: str,
        variant: str = "primary",
        width: int | None = None,
        height: int | None = None,
    ) -> QPushButton:
        button = QPushButton(text)
        button.setStyleSheet(self.button_style(variant))

        if width is not None and height is not None:
            button.setFixedSize(width, height)

        return button

    def section_title(self, text: str) -> QLabel:
        label = QLabel(text)
        label.setStyleSheet(
            f"""
            QLabel {{
                color: {self.COLOR_MUTED};
                font-weight: bold;
                font-size: 10px;
                margin-top: 4px;
            }}
            """
        )
        return label

    def info_box_style(self, monospace: bool = False) -> str:
        font = "Courier" if monospace else "Arial"
        return f"""
        QLabel {{
            background-color: {self.COLOR_PANEL_2};
            color: {self.COLOR_TEXT};
            padding: 10px;
            border: 1px solid {self.COLOR_BORDER};
            border-radius: 8px;
            font-family: {font};
            font-size: 10px;
        }}
        """

    def combo_style(self) -> str:
        return f"""
        QComboBox {{
            background-color: {self.COLOR_PANEL_2};
            color: {self.COLOR_TEXT};
            border: 1px solid {self.COLOR_BORDER};
            border-radius: 8px;
            padding: 8px;
        }}

        QComboBox:hover {{
            border: 1px solid {self.COLOR_ACCENT};
        }}

        QComboBox::drop-down {{
            border: none;
        }}

        QComboBox QAbstractItemView {{
            background-color: {self.COLOR_PANEL_2};
            color: {self.COLOR_TEXT};
            selection-background-color: {self.COLOR_ACCENT};
        }}
        """

    # ------------------------------------------------------------------
    # Thread-safe ROS data buffering
    # ------------------------------------------------------------------

    def enqueue_ros_update(self, data: Dict[str, Any]) -> None:
        with self._ros_data_lock:
            self._latest_ros_data = data
            self._has_pending_ros_data = True

    def process_pending_ros_update(self) -> None:
        with self._ros_data_lock:
            if not self._has_pending_ros_data:
                return

            data = self._latest_ros_data
            self._latest_ros_data = None
            self._has_pending_ros_data = False

        if data is None:
            return

        try:
            self.on_ros_update(data)
        except Exception as exc:
            self.append_log(f"Ошибка обновления GUI из ROS-данных: {exc}", "ERROR")

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def append_log(self, message: str, level: str = "INFO") -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        line = f"[{timestamp}] [{level}] {message}"

        if len(self._pending_log_lines) >= self._max_pending_log_lines:
            if level in ("ERROR", "WARN"):
                self._pending_log_lines.pop(0)
                self._pending_log_lines.append(line)
            else:
                self._dropped_log_lines += 1
            return

        self._pending_log_lines.append(line)

    def flush_log(self) -> None:
        if not hasattr(self, "log_display"):
            return

        lines_to_write = []

        if self._dropped_log_lines > 0:
            lines_to_write.append(
                f"[{datetime.now().strftime('%H:%M:%S')}] "
                f"[WARN] Log throttled: skipped {self._dropped_log_lines} repeated messages"
            )
            self._dropped_log_lines = 0

        if self._pending_log_lines:
            batch_size = 30
            lines_to_write.extend(self._pending_log_lines[:batch_size])
            del self._pending_log_lines[:batch_size]

        if not lines_to_write:
            return

        self.log_display.setUpdatesEnabled(False)
        self.log_display.appendPlainText("\n".join(lines_to_write))
        self.log_display.setUpdatesEnabled(True)

        scrollbar = self.log_display.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def clear_log(self) -> None:
        self._pending_log_lines.clear()
        self._dropped_log_lines = 0
        self.log_display.clear()

    def toggle_logs(self) -> None:
        self.logs_visible = not self.logs_visible
        self.log_panel.setVisible(self.logs_visible)

        if self.logs_visible:
            self.toggle_logs_btn.setText("Скрыть логи")
            self.append_log("Логи отображены.")
        else:
            self.toggle_logs_btn.setText("Показать логи")

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def current_speed_level(self) -> Dict[str, Any]:
        return self.speed_levels[self.speed_idx]

    def update_speed(self):
        if self.robot is not None:
            self.robot.speed_scale = self.current_speed_level()["scale"]

    def on_speed_changed(self, index: int):
        self.speed_idx = index
        self.update_speed()

        level = self.current_speed_level()
        self.append_log(
            "Скорость изменена: "
            f"{level['label']}, "
            f"linear={level['linear_speed_mps'] * 1000.0:.0f} мм/с, "
            f"angular={math.degrees(level['angular_speed_rad_s']):.1f} °/с"
        )

    def update_system_status(self, status: Dict[str, Any]) -> None:
        connected = bool(status.get("connected", False))
        joint_states = bool(status.get("joint_states", False))
        motion_state = str(status.get("motion_state", "unknown"))
        e_stop_active = bool(status.get("e_stop_active", False))
        gripper_target = status.get("gripper_target")

        connected_text = "OK" if connected else "NO"
        joint_states_text = "OK" if joint_states else "WAIT"
        estop_text = "ACTIVE" if e_stop_active else "OFF"

        gripper_target_text = "---"
        if gripper_target is not None:
            gripper_target_text = f"{float(gripper_target) * 1000.0:.1f} mm"

        self.system_status_display.setText(
            f"ROS: {connected_text}\n"
            f"Joint states: {joint_states_text}\n"
            f"Motion: {motion_state}\n"
            f"E-STOP: {estop_text}\n"
            f"Gripper target: {gripper_target_text}"
        )

        if e_stop_active:
            self.system_status_display.setStyleSheet(
                self.info_box_style()
                + f"""
                QLabel {{
                    border: 1px solid {self.COLOR_DANGER};
                }}
                """
            )
        else:
            self.system_status_display.setStyleSheet(self.info_box_style())

        if self.last_motion_state != motion_state:
            now = time.monotonic()
            important_motion_states = {"timeout", "estop", "stopping"}

            if motion_state in important_motion_states:
                self.append_log(f"Motion state: {motion_state}", "WARN")
                self.last_motion_log_time = now
            elif now - self.last_motion_log_time > 5.0 and motion_state not in {
                "moving",
                "done",
            }:
                self.append_log(f"Motion state: {motion_state}")
                self.last_motion_log_time = now

            self.last_motion_state = motion_state

        if self.last_estop_state != e_stop_active:
            state_text = "activated" if e_stop_active else "reset/off"
            self.append_log(
                f"Emergency Stop: {state_text}",
                "WARN" if e_stop_active else "INFO",
            )
            self.last_estop_state = e_stop_active

        if self.last_joint_states_state != joint_states:
            state_text = "received" if joint_states else "not received"
            self.append_log(f"Joint states: {state_text}")
            self.last_joint_states_state = joint_states

    def is_estop_active(self) -> bool:
        if self.robot is None:
            return False

        try:
            return bool(self.robot.get_status().get("e_stop_active", False))
        except Exception:
            return False

    # ------------------------------------------------------------------
    # Smoothed manual movement
    # ------------------------------------------------------------------

    @staticmethod
    def normalize_vector(dx: float, dy: float, dz: float) -> Vector3:
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length <= 1e-12:
            return (0.0, 0.0, 0.0)

        k = 1.0 / length
        return (dx * k, dy * k, dz * k)

    @staticmethod
    def vector_length(v: Vector3) -> float:
        return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

    @staticmethod
    def approach_vector(current: Vector3, target: Vector3, alpha: float) -> Vector3:
        alpha = max(0.0, min(1.0, alpha))
        return (
            current[0] + (target[0] - current[0]) * alpha,
            current[1] + (target[1] - current[1]) * alpha,
            current[2] + (target[2] - current[2]) * alpha,
        )

    def ensure_manual_timer_running(self) -> None:
        if not self.manual_timer.isActive():
            now = time.monotonic()
            self._last_manual_tick_time = now
            self._last_manual_send_time = 0.0
            self.manual_timer.start()

    def start_move(self, dx: float, dy: float, dz: float):
        if self.is_estop_active():
            self.append_log("Движение заблокировано: активен Emergency Stop.", "WARN")
            return

        self.current_move = self.normalize_vector(dx, dy, dz)
        self.ensure_manual_timer_running()

    def stop_move(self):
        # Не вызываем stop_motion(). Просто задаём цель 0,
        # а таймер плавно затормозит движение.
        self.current_move = (0.0, 0.0, 0.0)

    def start_rotate(self, drx: float, dry: float, drz: float):
        if self.is_estop_active():
            self.append_log("Поворот заблокирован: активен Emergency Stop.", "WARN")
            return

        self.current_drot = self.normalize_vector(drx, dry, drz)
        self.ensure_manual_timer_running()

    def stop_rotate(self):
        # Мягкое торможение поворота.
        self.current_drot = (0.0, 0.0, 0.0)

    def stop_all_manual_motion(self) -> None:
        self.current_move = (0.0, 0.0, 0.0)
        self.current_drot = (0.0, 0.0, 0.0)
        self.smooth_move = (0.0, 0.0, 0.0)
        self.smooth_drot = (0.0, 0.0, 0.0)
        self.manual_timer.stop()

    def manual_motion_is_idle(self) -> bool:
        return (
            self.vector_length(self.current_move) < 1e-3
            and self.vector_length(self.current_drot) < 1e-3
            and self.vector_length(self.smooth_move) < 1e-3
            and self.vector_length(self.smooth_drot) < 1e-3
        )

    def log_manual_error_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_manual_error_log_time >= 1.5:
            self.append_log(message, "ERROR")
            self._last_manual_error_log_time = now

    def on_manual_timer(self):
        if self.robot is None:
            self.stop_all_manual_motion()
            return

        if self.is_estop_active():
            self.stop_all_manual_motion()
            return

        now = time.monotonic()
        dt = now - self._last_manual_tick_time
        self._last_manual_tick_time = now

        if dt <= 0.0:
            dt = self.manual_timer_interval_ms / 1000.0

        dt = max(0.005, min(dt, 0.12))

        alpha = min(1.0, dt / self.manual_ramp_time_sec)

        self.smooth_move = self.approach_vector(
            self.smooth_move,
            self.current_move,
            alpha,
        )
        self.smooth_drot = self.approach_vector(
            self.smooth_drot,
            self.current_drot,
            alpha,
        )

        if self.manual_motion_is_idle():
            self.manual_timer.stop()
            return

        if now - self._last_manual_send_time < self.manual_send_period_sec:
            return

        self._last_manual_send_time = now

        level = self.current_speed_level()
        linear_speed = float(level["linear_speed_mps"])
        angular_speed = float(level["angular_speed_rad_s"])

        send_period = self.manual_send_period_sec
        duration = self.manual_trajectory_duration_sec

        move_len = self.vector_length(self.smooth_move)
        rot_len = self.vector_length(self.smooth_drot)

        try:
            # Если одновременно зажаты движение и поворот,
            # приоритет отдаём линейному движению, чтобы не посылать
            # две разные траектории в один и тот же момент.
            if move_len > 1e-3:
                dx = self.smooth_move[0] * linear_speed * send_period
                dy = self.smooth_move[1] * linear_speed * send_period
                dz = self.smooth_move[2] * linear_speed * send_period

                self.robot.move_end_effector_world(
                    dx,
                    dy,
                    dz,
                    duration=duration,
                )
                return

            if rot_len > 1e-3:
                drx = self.smooth_drot[0] * angular_speed * send_period
                dry = self.smooth_drot[1] * angular_speed * send_period
                drz = self.smooth_drot[2] * angular_speed * send_period

                if abs(drx) > 1e-6 or abs(dry) > 1e-6:
                    self.robot.rotate_end_effector_rx_ry_ik(
                        drx,
                        dry,
                        duration=duration,
                    )

                if abs(drz) > 1e-6:
                    self.robot.rotate_end_effector_world(
                        drz,
                        duration=duration,
                    )

        except Exception as exc:
            self.log_manual_error_throttled(f"Ошибка ручного движения: {exc}")
            self.stop_all_manual_motion()

    # ------------------------------------------------------------------
    # Buttons
    # ------------------------------------------------------------------

    def on_home(self):
        if self.robot is None:
            return

        self.stop_all_manual_motion()
        self.append_log("Команда: переход в исходную позицию.")
        self.robot.reset_position()

    def on_stop_motion(self):
        if self.robot is None:
            return

        self.stop_all_manual_motion()
        self.append_log("Команда: Stop Motion.", "WARN")
        self.robot.stop_motion()

    def on_emergency_stop(self):
        if self.robot is None:
            return

        self.stop_all_manual_motion()
        self.append_log("Команда: Emergency Stop.", "WARN")
        self.robot.emergency_stop()

    def on_reset_emergency_stop(self):
        if self.robot is None:
            return

        self.append_log("Команда: Reset Emergency Stop.")
        self.robot.reset_emergency_stop()

    def on_gripper_open(self):
        if self.robot is None:
            return

        self.append_log("Команда: открыть захват.")
        self.robot.open_gripper(0.7)

    def on_gripper_close(self):
        if self.robot is None:
            return

        self.append_log("Команда: закрыть захват.")
        self.robot.close_gripper(0.7)

    # ------------------------------------------------------------------
    # ROS update callback, executed only from GUI thread
    # ------------------------------------------------------------------

    def on_ros_update(self, data: Dict[str, Any]):
        if "status" in data:
            self.update_system_status(data["status"])

        if "position" in data:
            positions = data["position"]
            angles_deg = [value * (180.0 / math.pi) for value in positions]

            status_joints = ""
            for i, angle in enumerate(angles_deg, 1):
                status_joints += f"J{i}: {angle:7.2f}° "
                if i % 3 == 0:
                    status_joints += "\n"

            self.position_display.setText(status_joints.strip())

        if "pose" in data:
            pose = data["pose"]
            self.pose_display.setText(
                f"X: {pose['x'] * 1000.0:8.1f} mm "
                f"Y: {pose['y'] * 1000.0:8.1f} mm "
                f"Z: {pose['z'] * 1000.0:8.1f} mm\n"
                f"RX: {pose['rx']:8.1f}° "
                f"RY: {pose['ry']:8.1f}° "
                f"RZ: {pose['rz']:8.1f}°"
            )

        if "gripper" in data:
            gripper = data["gripper"]
            if gripper:
                opening_m = max(gripper)
                opening_mm = opening_m * 1000.0

                if opening_m < 0.003:
                    state = "закрыт"
                elif opening_m > 0.022:
                    state = "открыт"
                else:
                    state = "промежуточно"

                self.gripper_display.setText(
                    f"Захват: {opening_mm:5.1f} mm | {state}"
                )

    # ------------------------------------------------------------------
    # Gazebo
    # ------------------------------------------------------------------

    def find_workspace_root(self) -> Path:
        current_file = Path(__file__).resolve()

        for parent in [current_file.parent] + list(current_file.parents):
            gazebo_launch = (
                parent
                / "src"
                / "robot_gazebo"
                / "launch"
                / "gazebo.launch.py"
            )
            if gazebo_launch.exists():
                return parent

        return Path.home() / "RobotManipulator"

    def find_free_gazebo_port(self, start_port: int = 11345) -> int:
        for port in range(start_port, start_port + 100):
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                sock.bind(("127.0.0.1", port))
                sock.close()
                return port
            except OSError:
                sock.close()

        return start_port

    def cleanup_gazebo_processes(self) -> None:
        """
        Безопасная очистка Gazebo.

        Важно:
        не используем pkill -f "gazebo" напрямую, потому что такая команда
        может совпасть с самой cleanup-командой. Используем regex-шаблоны
        вида [g]azebo.
        """
        cleanup_command = r"""
            pkill -TERM -f '[r]os2 launch robot_gazebo' || true
            pkill -TERM -f '[g]zserver' || true
            pkill -TERM -f '[g]zclient' || true
            pkill -TERM -f '[g]azebo' || true
            pkill -TERM -f '[s]pawn_entity.py' || true
            pkill -TERM -f '[s]pawner' || true
            pkill -TERM -f '[r]os2 topic pub --once /arm_controller' || true
            pkill -TERM -f '[r]os2 topic pub --once /gripper_controller' || true
            sleep 1.5
            pkill -KILL -f '[r]os2 launch robot_gazebo' || true
            pkill -KILL -f '[g]zserver' || true
            pkill -KILL -f '[g]zclient' || true
            pkill -KILL -f '[g]azebo' || true
            pkill -KILL -f '[s]pawn_entity.py' || true
            pkill -KILL -f '[s]pawner' || true
            pkill -KILL -f '[r]os2 topic pub --once /arm_controller' || true
            pkill -KILL -f '[r]os2 topic pub --once /gripper_controller' || true
            sleep 1.5
        """

        try:
            subprocess.run(
                ["bash", "-lc", cleanup_command],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=8,
                check=False,
            )
        except Exception as exc:
            self.append_log(f"Ошибка при очистке Gazebo: {exc}", "ERROR")

    def on_start_gazebo(self):
        if (
            self.gazebo_process is not None
            and self.gazebo_process.state() != QProcess.ProcessState.NotRunning
        ):
            self.append_log("Gazebo уже запущен.", "WARN")
            return

        self.append_log("Подготовка Gazebo к запуску.")
        self.cleanup_gazebo_processes()

        workspace_root = self.find_workspace_root()
        setup_file = workspace_root / "install" / "setup.bash"

        if not setup_file.exists():
            self.append_log("install/setup.bash не найден.", "ERROR")
            self.append_log(
                "Выполни: cd ~/RobotManipulator && colcon build && source install/setup.bash"
            )
            return

        gazebo_port = self.find_free_gazebo_port(11345)
        gazebo_master_uri = f"http://127.0.0.1:{gazebo_port}"
        self.gazebo_master_uri = gazebo_master_uri

        if gazebo_port == 11345:
            self.append_log("Gazebo будет запущен на стандартном порту 11345.")
        else:
            self.append_log(
                f"Порт 11345 занят. Gazebo будет запущен на порту {gazebo_port}.",
                "WARN",
            )

        command = (
            f'cd "{workspace_root}" && '
            f"export GAZEBO_MASTER_URI={gazebo_master_uri} && "
            f"source /opt/ros/humble/setup.bash && "
            f"source install/setup.bash && "
            f"ros2 launch robot_gazebo gazebo.launch.py"
        )

        self.gazebo_process = QProcess(self)
        self.gazebo_process.setProgram("setsid")
        self.gazebo_process.setArguments(["bash", "-lc", command])
        self.gazebo_process.setProcessChannelMode(
            QProcess.ProcessChannelMode.MergedChannels
        )
        self.gazebo_process.readyReadStandardOutput.connect(self.on_gazebo_output)
        self.gazebo_process.finished.connect(self.on_gazebo_finished)

        self.gazebo_process.start()

        if not self.gazebo_process.waitForStarted(3000):
            self.append_log("Не удалось запустить Gazebo.", "ERROR")
            self.gazebo_process = None
            self.gazebo_pid = None
            self.gazebo_master_uri = None
            return

        self.gazebo_pid = int(self.gazebo_process.processId())

        self.append_log(f"Gazebo запускается. GAZEBO_MASTER_URI={gazebo_master_uri}")
        self.append_log("Подожди 15–20 секунд, пока загрузятся робот и контроллеры.")

    def on_stop_gazebo(self):
        self.append_log("Остановка Gazebo.", "WARN")
        self.stop_gazebo_process()

    def stop_gazebo_process(self):
        if self.gazebo_process is not None:
            if self.gazebo_process.state() != QProcess.ProcessState.NotRunning:
                if self.gazebo_pid is not None:
                    try:
                        os.killpg(os.getpgid(self.gazebo_pid), signal.SIGTERM)
                        self.append_log("SIGTERM отправлен группе процессов Gazebo.")
                    except ProcessLookupError:
                        pass
                    except Exception as exc:
                        self.append_log(
                            f"Не удалось отправить SIGTERM Gazebo: {exc}",
                            "ERROR",
                        )

                self.gazebo_process.waitForFinished(2000)

        self.gazebo_process = None
        self.gazebo_pid = None
        self.gazebo_master_uri = None

        self.cleanup_gazebo_processes()
        self.append_log("Gazebo остановлен. Можно запускать снова.")

    def on_gazebo_finished(self, exit_code=0, exit_status=None):
        self.append_log(f"Процесс Gazebo завершён. exit_code={exit_code}")
        self.gazebo_process = None
        self.gazebo_pid = None
        self.gazebo_master_uri = None

    def on_gazebo_output(self):
        if self.gazebo_process is None:
            return

        output = bytes(self.gazebo_process.readAllStandardOutput()).decode(
            errors="ignore"
        )
        text = output.strip()

        if not text:
            return

        important_lines = []

        for line in text.splitlines():
            lower = line.lower()

            if (
                "error" in lower
                or "err]" in lower
                or "warn" in lower
                or "failed" in lower
                or "exception" in lower
                or "traceback" in lower
                or "address already in use" in lower
                or "unable to start server" in lower
                or "controller_manager" in lower
            ):
                important_lines.append(line)

        if not important_lines:
            return

        now = time.monotonic()
        if now - self.last_gazebo_log_time < 1.0:
            return

        self.last_gazebo_log_time = now

        for line in important_lines[-3:]:
            lower = line.lower()
            level = (
                "ERROR"
                if (
                    "error" in lower
                    or "err]" in lower
                    or "failed" in lower
                    or "exception" in lower
                    or "address already in use" in lower
                    or "unable to start server" in lower
                )
                else "WARN"
            )
            self.append_log(f"Gazebo: {line}", level)