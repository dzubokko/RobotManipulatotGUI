from datetime import datetime
import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict

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


class ManualControlUI(QWidget):
    """
    Manual control panel for the robot manipulator.

    Responsibilities:
    - manual TCP movement;
    - TCP orientation movement;
    - gripper open/close;
    - Gazebo start/stop;
    - engineering status display;
    - GUI log panel;
    - Stop Motion and Emergency Stop buttons.

    Important:
    ROS callbacks may be called from a non-Qt thread.
    Therefore, this UI does not update widgets directly from ROS callbacks.
    Incoming ROS data is buffered and processed by a Qt timer.
    """

    def __init__(self, robot_bridge):
        super().__init__()

        self.robot = robot_bridge

        self.gazebo_process = None
        self.gazebo_pid = None

        self.speed_levels = [
            {"label": "10 мм/с", "scale": 0.5},
            {"label": "50 мм/с", "scale": 2.5},
            {"label": "100 мм/с", "scale": 5.0},
        ]
        self.speed_idx = 1

        self.current_move = (0.0, 0.0, 0.0)
        self.current_drot = (0.0, 0.0, 0.0)

        self.angle_step = 0.03

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

        self.move_timer = QTimer(self)
        self.move_timer.setInterval(50)
        self.move_timer.timeout.connect(self.on_move_timer)

        self.rotate_timer = QTimer(self)
        self.rotate_timer.setInterval(50)
        self.rotate_timer.timeout.connect(self.on_rotate_timer)

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
        self.append_log("Основной запуск проекта: python3 -m robot_arm_controller.app")

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def init_ui(self):
        main_layout = QVBoxLayout(self)

        title = QLabel("РУЧНОЕ УПРАВЛЕНИЕ РОБОТОМ-МАНИПУЛЯТОРОМ")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setStyleSheet("color: #0078d4; margin-bottom: 10px;")
        main_layout.addWidget(title)

        content = QHBoxLayout()

        left_panel = self.build_left_control_panel()
        right_panel = self.build_right_status_panel()
        log_panel = self.build_log_panel()

        content.addWidget(left_panel, 1)
        content.addWidget(right_panel, 1)
        content.addWidget(log_panel, 1)

        main_layout.addLayout(content)
        self.setLayout(main_layout)

    def build_left_control_panel(self) -> QGroupBox:
        left_group = QGroupBox("РУЧНОЕ ДВИЖЕНИЕ TCP")
        left_layout = QVBoxLayout(left_group)

        linear_group = QGroupBox("ЛИНЕЙНОЕ ДВИЖЕНИЕ")
        linear_layout = QGridLayout(linear_group)

        z_plus = QPushButton("Z+")
        z_plus.setFixedSize(90, 60)
        z_plus.setStyleSheet(self.get_btn_style("#0078d4"))
        z_plus.pressed.connect(lambda: self.start_move(0.0, 0.0, +1.0))
        z_plus.released.connect(self.stop_move)
        linear_layout.addWidget(z_plus, 0, 1)

        z_minus = QPushButton("Z-")
        z_minus.setFixedSize(90, 60)
        z_minus.setStyleSheet(self.get_btn_style("#0078d4"))
        z_minus.pressed.connect(lambda: self.start_move(0.0, 0.0, -1.0))
        z_minus.released.connect(self.stop_move)
        linear_layout.addWidget(z_minus, 2, 1)

        y_plus = QPushButton("Y+")
        y_plus.setFixedSize(90, 60)
        y_plus.setStyleSheet(self.get_btn_style("#28a745"))
        y_plus.pressed.connect(lambda: self.start_move(0.0, -1.0, 0.0))
        y_plus.released.connect(self.stop_move)
        linear_layout.addWidget(y_plus, 1, 2)

        y_minus = QPushButton("Y-")
        y_minus.setFixedSize(90, 60)
        y_minus.setStyleSheet(self.get_btn_style("#28a745"))
        y_minus.pressed.connect(lambda: self.start_move(0.0, +1.0, 0.0))
        y_minus.released.connect(self.stop_move)
        linear_layout.addWidget(y_minus, 1, 0)

        x_plus = QPushButton("X+")
        x_plus.setFixedSize(90, 60)
        x_plus.setStyleSheet(self.get_btn_style("#dc3545"))
        x_plus.pressed.connect(lambda: self.start_move(-1.0, 0.0, 0.0))
        x_plus.released.connect(self.stop_move)
        linear_layout.addWidget(x_plus, 3, 1)

        x_minus = QPushButton("X-")
        x_minus.setFixedSize(90, 60)
        x_minus.setStyleSheet(self.get_btn_style("#dc3545"))
        x_minus.pressed.connect(lambda: self.start_move(+1.0, 0.0, 0.0))
        x_minus.released.connect(self.stop_move)
        linear_layout.addWidget(x_minus, 4, 1)

        left_layout.addWidget(linear_group)

        angular_group = QGroupBox("УГЛОВОЙ ПОВОРОТ TCP")
        angular_layout = QGridLayout(angular_group)

        rx_minus = QPushButton("RX-")
        rx_minus.setFixedSize(90, 50)
        rx_minus.setStyleSheet(self.get_btn_style("#ffc107"))
        rx_minus.pressed.connect(lambda: self.start_rotate(+1.0, 0.0, 0.0))
        rx_minus.released.connect(self.stop_rotate)
        angular_layout.addWidget(rx_minus, 0, 0)

        rx_plus = QPushButton("RX+")
        rx_plus.setFixedSize(90, 50)
        rx_plus.setStyleSheet(self.get_btn_style("#ffc107"))
        rx_plus.pressed.connect(lambda: self.start_rotate(-1.0, 0.0, 0.0))
        rx_plus.released.connect(self.stop_rotate)
        angular_layout.addWidget(rx_plus, 0, 1)

        ry_minus = QPushButton("RY-")
        ry_minus.setFixedSize(90, 50)
        ry_minus.setStyleSheet(self.get_btn_style("#ffc107"))
        ry_minus.pressed.connect(lambda: self.start_rotate(0.0, +1.0, 0.0))
        ry_minus.released.connect(self.stop_rotate)
        angular_layout.addWidget(ry_minus, 1, 0)

        ry_plus = QPushButton("RY+")
        ry_plus.setFixedSize(90, 50)
        ry_plus.setStyleSheet(self.get_btn_style("#ffc107"))
        ry_plus.pressed.connect(lambda: self.start_rotate(0.0, -1.0, 0.0))
        ry_plus.released.connect(self.stop_rotate)
        angular_layout.addWidget(ry_plus, 1, 1)

        rz_minus = QPushButton("RZ-")
        rz_minus.setFixedSize(90, 50)
        rz_minus.setStyleSheet(self.get_btn_style("#17a2b8"))
        rz_minus.pressed.connect(lambda: self.start_rotate(0.0, 0.0, +1.0))
        rz_minus.released.connect(self.stop_rotate)
        angular_layout.addWidget(rz_minus, 2, 0)

        rz_plus = QPushButton("RZ+")
        rz_plus.setFixedSize(90, 50)
        rz_plus.setStyleSheet(self.get_btn_style("#17a2b8"))
        rz_plus.pressed.connect(lambda: self.start_rotate(0.0, 0.0, -1.0))
        rz_plus.released.connect(self.stop_rotate)
        angular_layout.addWidget(rz_plus, 2, 1)

        left_layout.addWidget(angular_group)

        safety_group = QGroupBox("БЕЗОПАСНОСТЬ")
        safety_layout = QVBoxLayout(safety_group)

        stop_btn = QPushButton("STOP MOTION")
        stop_btn.setStyleSheet(self.get_btn_style("#fd7e14"))
        stop_btn.clicked.connect(self.on_stop_motion)
        safety_layout.addWidget(stop_btn)

        estop_btn = QPushButton("EMERGENCY STOP")
        estop_btn.setStyleSheet(self.get_btn_style("#dc3545"))
        estop_btn.clicked.connect(self.on_emergency_stop)
        safety_layout.addWidget(estop_btn)

        reset_estop_btn = QPushButton("RESET E-STOP")
        reset_estop_btn.setStyleSheet(self.get_btn_style("#28a745"))
        reset_estop_btn.clicked.connect(self.on_reset_emergency_stop)
        safety_layout.addWidget(reset_estop_btn)

        left_layout.addWidget(safety_group)
        left_layout.addStretch()

        return left_group

    def build_right_status_panel(self) -> QGroupBox:
        right_group = QGroupBox("СТАТУС И УПРАВЛЕНИЕ")
        right_layout = QVBoxLayout(right_group)

        status_title = QLabel("СИСТЕМНЫЙ СТАТУС")
        status_title.setStyleSheet(self.get_title_label_style())
        right_layout.addWidget(status_title)

        self.system_status_display = QLabel(
            "ROS: ---\n"
            "Joint states: ---\n"
            "Motion: ---\n"
            "E-STOP: ---"
        )
        self.system_status_display.setStyleSheet(self.get_status_style(False))
        right_layout.addWidget(self.system_status_display)

        joints_title = QLabel("ТЕКУЩИЕ УГЛЫ СУСТАВОВ")
        joints_title.setStyleSheet(self.get_title_label_style())
        right_layout.addWidget(joints_title)

        self.position_display = QLabel(
            "J1: 0.0° J2: 0.0° J3: 0.0°\n"
            "J4: 0.0° J5: 0.0° J6: 0.0°"
        )
        self.position_display.setStyleSheet(
            "background-color: #1e1e1e; "
            "color: #00ff00; "
            "padding: 10px; "
            "border: 1px solid #0078d4; "
            "border-radius: 4px; "
            "font-family: Courier; "
            "font-size: 10px;"
        )
        right_layout.addWidget(self.position_display)

        pose_title = QLabel("КООРДИНАТЫ TCP")
        pose_title.setStyleSheet(self.get_title_label_style())
        right_layout.addWidget(pose_title)

        self.pose_display = QLabel(
            "X: --- mm Y: --- mm Z: --- mm\n"
            "RX: ---° RY: ---° RZ: ---°"
        )
        self.pose_display.setStyleSheet(
            "background-color: #1a1a2e; "
            "color: #00ff88; "
            "padding: 10px; "
            "border: 2px solid #00cc66; "
            "border-radius: 6px; "
            "font-family: Courier; "
            "font-size: 10px;"
        )
        right_layout.addWidget(self.pose_display)

        gripper_title = QLabel("СОСТОЯНИЕ ЗАХВАТА")
        gripper_title.setStyleSheet(self.get_title_label_style())
        right_layout.addWidget(gripper_title)

        self.gripper_display = QLabel("Захват: --- mm")
        self.gripper_display.setStyleSheet(
            "background-color: #1e1e1e; "
            "color: #ffaa00; "
            "padding: 8px; "
            "border: 1px solid #ffaa00; "
            "border-radius: 4px; "
            "font-family: Courier; "
            "font-size: 10px;"
        )
        right_layout.addWidget(self.gripper_display)

        speed_title = QLabel("СКОРОСТЬ ПОДАЧИ")
        speed_title.setStyleSheet(self.get_title_label_style("#ff6600"))
        right_layout.addWidget(speed_title)

        self.speed_combo = QComboBox()
        for level in self.speed_levels:
            self.speed_combo.addItem(level["label"])
        self.speed_combo.setCurrentIndex(self.speed_idx)
        self.speed_combo.currentIndexChanged.connect(self.on_speed_changed)
        self.speed_combo.setStyleSheet(
            "QComboBox { "
            "padding: 6px; "
            "border-radius: 4px; "
            "background-color: #333333; "
            "color: white; "
            "border: 1px solid #555555; "
            "} "
            "QComboBox::drop-down { border: none; } "
            "QComboBox::down-arrow { image: none; }"
        )
        right_layout.addWidget(self.speed_combo)

        home_btn = QPushButton("ИСХОДНАЯ ПОЗИЦИЯ")
        home_btn.setStyleSheet(self.get_btn_style("#28a745"))
        home_btn.clicked.connect(self.on_home)
        right_layout.addWidget(home_btn)

        fk_tf_btn = QPushButton("ПРОВЕРИТЬ FK/TF")
        fk_tf_btn.setStyleSheet(self.get_btn_style("#6f42c1"))
        fk_tf_btn.clicked.connect(self.on_check_fk_tf)
        right_layout.addWidget(fk_tf_btn)

        gripper_group = QGroupBox("ЗАХВАТ")
        gripper_layout = QHBoxLayout(gripper_group)

        open_gripper_btn = QPushButton("ОТКРЫТЬ")
        open_gripper_btn.setStyleSheet(self.get_btn_style("#17a2b8"))
        open_gripper_btn.clicked.connect(self.on_gripper_open)
        gripper_layout.addWidget(open_gripper_btn)

        close_gripper_btn = QPushButton("ЗАКРЫТЬ")
        close_gripper_btn.setStyleSheet(self.get_btn_style("#dc3545"))
        close_gripper_btn.clicked.connect(self.on_gripper_close)
        gripper_layout.addWidget(close_gripper_btn)

        right_layout.addWidget(gripper_group)

        simulation_group = QGroupBox("СИМУЛЯЦИЯ")
        simulation_layout = QVBoxLayout(simulation_group)

        start_sim_btn = QPushButton("ОТКРЫТЬ СИМУЛЯЦИЮ")
        start_sim_btn.setStyleSheet(self.get_btn_style("#6f42c1"))
        start_sim_btn.clicked.connect(self.on_start_gazebo)
        simulation_layout.addWidget(start_sim_btn)

        stop_sim_btn = QPushButton("ОСТАНОВИТЬ СИМУЛЯЦИЮ")
        stop_sim_btn.setStyleSheet(self.get_btn_style("#fd7e14"))
        stop_sim_btn.clicked.connect(self.on_stop_gazebo)
        simulation_layout.addWidget(stop_sim_btn)

        right_layout.addWidget(simulation_group)
        right_layout.addStretch()

        return right_group

    def build_log_panel(self) -> QGroupBox:
        log_group = QGroupBox("ЖУРНАЛ СОБЫТИЙ")
        log_layout = QVBoxLayout(log_group)

        self.log_display = QPlainTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setMaximumBlockCount(200)
        self.log_display.setStyleSheet(
            "background-color: #111111; "
            "color: #d7ffd7; "
            "border: 1px solid #333333; "
            "border-radius: 4px; "
            "padding: 8px; "
            "font-family: Courier; "
            "font-size: 10px;"
        )
        log_layout.addWidget(self.log_display)

        clear_log_btn = QPushButton("ОЧИСТИТЬ ЛОГ")
        clear_log_btn.setStyleSheet(self.get_btn_style("#555555"))
        clear_log_btn.clicked.connect(self.clear_log)
        log_layout.addWidget(clear_log_btn)

        return log_group

    # ------------------------------------------------------------------
    # Styles
    # ------------------------------------------------------------------

    def get_btn_style(self, color: str) -> str:
        return f"""
        QPushButton {{
            background-color: {color};
            color: white;
            border: 2px solid #005a9e;
            border-radius: 8px;
            font-weight: bold;
            font-size: 10px;
            padding: 7px;
        }}
        QPushButton:hover {{
            background-color: {color}dd;
            border: 2px solid #0078d4;
        }}
        QPushButton:pressed {{
            background-color: {color}88;
            border: 2px solid #005a9e;
        }}
        QPushButton:focus {{
            outline: none;
            border: 2px solid #0078d4;
        }}
        """

    @staticmethod
    def get_title_label_style(color: str = "#0078d4") -> str:
        return (
            f"font-weight: bold; "
            f"color: {color}; "
            f"font-size: 11px; "
            f"margin-top: 8px;"
        )

    @staticmethod
    def get_status_style(estop_active: bool) -> str:
        if estop_active:
            return (
                "background-color: #2a0000; "
                "color: #ffdddd; "
                "padding: 10px; "
                "border: 2px solid #dc3545; "
                "border-radius: 6px; "
                "font-family: Courier; "
                "font-size: 10px;"
            )

        return (
            "background-color: #1e1e1e; "
            "color: #ffffff; "
            "padding: 10px; "
            "border: 2px solid #0078d4; "
            "border-radius: 6px; "
            "font-family: Courier; "
            "font-size: 10px;"
        )

    # ------------------------------------------------------------------
    # Thread-safe ROS data buffering
    # ------------------------------------------------------------------

    def enqueue_ros_update(self, data: Dict[str, Any]) -> None:
        """
        Called by RobotBridge from ROS thread.

        Do not update Qt widgets here.
        Store only the latest packet.
        """
        with self._ros_data_lock:
            self._latest_ros_data = data
            self._has_pending_ros_data = True

    def process_pending_ros_update(self) -> None:
        """
        Called by Qt timer from GUI thread.
        Safe place to update widgets.
        """
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

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def update_speed(self):
        if self.robot is not None:
            self.robot.speed_scale = self.speed_levels[self.speed_idx]["scale"]

    def on_speed_changed(self, index: int):
        self.speed_idx = index
        self.update_speed()

        label = self.speed_levels[self.speed_idx]["label"]
        scale = self.speed_levels[self.speed_idx]["scale"]
        self.append_log(f"Скорость изменена: {label}, scale={scale:.2f}")

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
        self.system_status_display.setStyleSheet(self.get_status_style(e_stop_active))

        if self.last_motion_state != motion_state:
            now = time.monotonic()
            important_motion_states = {"timeout", "estop", "stopping"}

            if motion_state in important_motion_states:
                self.append_log(f"Motion state: {motion_state}", "WARN")
                self.last_motion_log_time = now
            elif (
                now - self.last_motion_log_time > 5.0
                and motion_state not in {"moving", "done"}
            ):
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
    # Movement
    # ------------------------------------------------------------------

    def start_move(self, dx: float, dy: float, dz: float):
        if self.is_estop_active():
            self.append_log("Движение заблокировано: активен Emergency Stop.", "WARN")
            return

        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length == 0.0:
            return

        k = 1.0 / length
        self.current_move = (dx * k, dy * k, dz * k)

        if not self.move_timer.isActive():
            self.move_timer.start()

    def stop_move(self):
        self.current_move = (0.0, 0.0, 0.0)
        self.move_timer.stop()

    def start_rotate(self, drx: float, dry: float, drz: float):
        if self.is_estop_active():
            self.append_log("Поворот заблокирован: активен Emergency Stop.", "WARN")
            return

        length = math.sqrt(drx * drx + dry * dry + drz * drz)
        if length == 0.0:
            return

        k = 1.0 / length
        self.current_drot = (drx * k, dry * k, drz * k)

        if not self.rotate_timer.isActive():
            self.rotate_timer.start()

    def stop_rotate(self):
        self.current_drot = (0.0, 0.0, 0.0)
        self.rotate_timer.stop()

    def on_move_timer(self):
        dx, dy, dz = self.current_move
        if dx == dy == dz == 0.0:
            return

        if self.robot is None:
            return

        if self.is_estop_active():
            self.stop_move()
            return

        base_step_m = 0.001
        scaled_step_size = base_step_m * self.robot.speed_scale

        try:
            self.robot.move_end_effector_world(
                dx * scaled_step_size,
                dy * scaled_step_size,
                dz * scaled_step_size,
                duration=0.05,
            )
        except Exception as exc:
            self.append_log(f"Ошибка линейного движения: {exc}", "ERROR")
            self.stop_move()

    def on_rotate_timer(self):
        drx, dry, drz = self.current_drot
        if drx == dry == drz == 0.0:
            return

        if self.robot is None:
            return

        if self.is_estop_active():
            self.stop_rotate()
            return

        angle_delta = self.angle_step

        try:
            if drx != 0.0 or dry != 0.0:
                self.robot.rotate_end_effector_rx_ry_ik(
                    drx * angle_delta,
                    dry * angle_delta,
                    duration=0.05,
                )

            if drz != 0.0:
                self.robot.rotate_end_effector_world(
                    drz * angle_delta,
                    duration=0.05,
                )

        except Exception as exc:
            self.append_log(f"Ошибка углового движения: {exc}", "ERROR")
            self.stop_rotate()

    # ------------------------------------------------------------------
    # Buttons
    # ------------------------------------------------------------------

    def on_home(self):
        if self.robot is None:
            return

        self.stop_move()
        self.stop_rotate()

        self.append_log("Команда: переход в исходную позицию.")
        self.robot.reset_position()

    def on_stop_motion(self):
        if self.robot is None:
            return

        self.stop_move()
        self.stop_rotate()

        self.append_log("Команда: Stop Motion.", "WARN")
        self.robot.stop_motion()

    def on_emergency_stop(self):
        if self.robot is None:
            return

        self.stop_move()
        self.stop_rotate()

        self.append_log("Команда: Emergency Stop.", "WARN")
        self.robot.emergency_stop()

    def on_reset_emergency_stop(self):
        if self.robot is None:
            return

        self.append_log("Команда: Reset Emergency Stop.")
        self.robot.reset_emergency_stop()

    def on_check_fk_tf(self):
        if self.robot is None:
            return

        self.append_log("Запуск проверки FK/TF.")

        result = self.robot.check_fk_against_tf()
        if result is None:
            self.append_log("FK/TF проверка не выполнена. TF недоступен.", "WARN")
            return

        self.append_log(
            "FK/TF: "
            f"FK=({result['fk_x']:.4f}, {result['fk_y']:.4f}, {result['fk_z']:.4f}) m | "
            f"TF=({result['tf_x']:.4f}, {result['tf_y']:.4f}, {result['tf_z']:.4f}) m | "
            f"error=({result['error_x_mm']:.2f}, "
            f"{result['error_y_mm']:.2f}, "
            f"{result['error_z_mm']:.2f}) mm | "
            f"total={result['error_total_mm']:.2f} mm"
        )

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
                parent / "src" / "robot_gazebo" / "launch" / "gazebo.launch.py"
            )
            if gazebo_launch.exists():
                return parent

        return Path.home() / "RobotManipulator"

    def on_start_gazebo(self):
        if (
            self.gazebo_process is not None
            and self.gazebo_process.state() != QProcess.ProcessState.NotRunning
        ):
            self.append_log("Gazebo уже запущен.", "WARN")
            return

        workspace_root = self.find_workspace_root()
        setup_file = workspace_root / "install" / "setup.bash"

        if not setup_file.exists():
            self.append_log("install/setup.bash не найден.", "ERROR")
            self.append_log(
                "Выполни: cd ~/RobotManipulator && colcon build && source install/setup.bash"
            )
            return

        command = (
            f'cd "{workspace_root}" && '
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
            return

        self.gazebo_pid = int(self.gazebo_process.processId())
        self.append_log("Gazebo запускается.")
        self.append_log("Подожди 10–15 секунд, пока загрузятся робот и контроллеры.")

    def on_stop_gazebo(self):
        self.append_log("Остановка Gazebo.", "WARN")
        self.stop_gazebo_process()

    def stop_gazebo_process(self):
        if self.gazebo_process is None:
            self.cleanup_gazebo_processes()
            return

        if self.gazebo_process.state() == QProcess.ProcessState.NotRunning:
            self.cleanup_gazebo_processes()
            self.gazebo_process = None
            self.gazebo_pid = None
            return

        if self.gazebo_pid is not None:
            try:
                os.killpg(os.getpgid(self.gazebo_pid), signal.SIGTERM)
                self.append_log("SIGTERM отправлен группе процессов Gazebo.")
            except ProcessLookupError:
                self.append_log("Группа процессов Gazebo уже завершена.")
            except Exception as exc:
                self.append_log(f"Не удалось остановить группу Gazebo: {exc}", "ERROR")

        if not self.gazebo_process.waitForFinished(5000):
            self.append_log("Мягкая остановка Gazebo не сработала. SIGKILL.", "WARN")

            if self.gazebo_pid is not None:
                try:
                    os.killpg(os.getpgid(self.gazebo_pid), signal.SIGKILL)
                    self.append_log("SIGKILL отправлен группе процессов Gazebo.")
                except Exception as exc:
                    self.append_log(
                        f"Не удалось принудительно завершить Gazebo: {exc}",
                        "ERROR",
                    )

            self.gazebo_process.kill()
            self.gazebo_process.waitForFinished(3000)

        self.cleanup_gazebo_processes()
        self.gazebo_process = None
        self.gazebo_pid = None
        self.append_log("Gazebo остановлен.")

    def cleanup_gazebo_processes(self):
        cleanup_command = (
            "pkill -TERM -f 'ros2 launch robot_gazebo gazebo.launch.py' || true; "
            "pkill -TERM -f 'gzserver' || true; "
            "pkill -TERM -f 'gzclient' || true; "
            "pkill -TERM -f 'gazebo' || true"
        )

        try:
            subprocess.run(
                ["bash", "-lc", cleanup_command],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3,
                check=False,
            )
        except Exception as exc:
            self.append_log(f"Ошибка при очистке Gazebo-процессов: {exc}", "ERROR")

    def on_gazebo_finished(self, exit_code=0, exit_status=None):
        self.append_log(f"Процесс Gazebo завершён. exit_code={exit_code}")
        self.gazebo_process = None
        self.gazebo_pid = None

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
                or "warn" in lower
                or "failed" in lower
                or "exception" in lower
                or "traceback" in lower
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
            level = "ERROR" if "error" in lower or "failed" in lower else "WARN"
            self.append_log(f"Gazebo: {line}", level)