from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QGridLayout,
    QComboBox,
)
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QFont

from typing import Dict
import math


class ManualControlUI(QWidget):
    """
    Стабильная панель ручного управления.

    Сейчас включено:
    - запуск/остановка Gazebo;
    - HOME;
    - суставное управление J1...J6;
    - открыть/закрыть захват;
    - отображение углов суставов и TCP.

    Временно отключено:
    - X/Y/Z;
    - RX/RY.

    Почему отключено:
    самописная IK-кинематика сейчас нестабильна.
    Дальше эти кнопки подключим через MoveIt/MoveIt Servo.
    """

    def __init__(self, robot_bridge):
        super().__init__()

        self.robot = robot_bridge

        self.speed_levels = [
            {"label": "Быстро", "scale": 1.0, "joint_step": math.radians(10)},
            {"label": "Средне", "scale": 0.5, "joint_step": math.radians(5)},
            {"label": "Медленно", "scale": 0.2, "joint_step": math.radians(1)},
        ]

        self.speed_idx = 1
        self.joint_step = self.speed_levels[self.speed_idx]["joint_step"]

        self.move_timer = QTimer(self)
        self.move_timer.setInterval(50)
        self.move_timer.timeout.connect(self.on_move_timer)
        self.current_move = (0.0, 0.0, 0.0)

        self.rotate_timer = QTimer(self)
        self.rotate_timer.setInterval(50)
        self.rotate_timer.timeout.connect(self.on_rotate_timer)
        self.current_drot = (0.0, 0.0, 0.0)

        self.angle_step = 0.03

        self.init_ui()

        if self.robot is not None:
            self.robot.register_callback(self.on_ros_update)

    # ============================================================
    # UI
    # ============================================================

    def init_ui(self):
        main_layout = QVBoxLayout(self)

        title = QLabel("РУЧНОЕ УПРАВЛЕНИЕ РОБОТОМ")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setStyleSheet("color: #0078d4; margin-bottom: 15px;")
        main_layout.addWidget(title)

        content = QHBoxLayout()

        left_panel = QVBoxLayout()
        right_panel = QVBoxLayout()

        # ========================================================
        # Simulation group
        # ========================================================

        simulation_group = QGroupBox("СИМУЛЯЦИЯ")
        simulation_layout = QGridLayout(simulation_group)

        start_sim_btn = QPushButton("ЗАПУСТИТЬ GAZEBO")
        start_sim_btn.setFixedHeight(42)
        start_sim_btn.setStyleSheet(self.get_btn_style("#28a745"))
        start_sim_btn.clicked.connect(self.on_start_simulation)
        simulation_layout.addWidget(start_sim_btn, 0, 0)

        stop_sim_btn = QPushButton("ОСТАНОВИТЬ GAZEBO")
        stop_sim_btn.setFixedHeight(42)
        stop_sim_btn.setStyleSheet(self.get_btn_style("#dc3545"))
        stop_sim_btn.clicked.connect(self.on_stop_simulation)
        simulation_layout.addWidget(stop_sim_btn, 0, 1)

        self.sim_status_label = QLabel("Статус: неизвестно")
        self.sim_status_label.setStyleSheet(
            "background-color: #1e1e1e; "
            "color: #ffaa00; "
            "padding: 8px; "
            "border: 1px solid #555; "
            "border-radius: 4px;"
        )
        simulation_layout.addWidget(self.sim_status_label, 1, 0, 1, 2)

        left_panel.addWidget(simulation_group)

        # ========================================================
        # Joint control group
        # ========================================================

        joint_group = QGroupBox("СУСТАВНОЕ УПРАВЛЕНИЕ")
        joint_layout = QGridLayout(joint_group)

        for i in range(6):
            joint_label = QLabel(f"J{i + 1}")
            joint_label.setStyleSheet(
                "font-weight: bold; color: #ffffff; font-size: 12px;"
            )
            joint_layout.addWidget(joint_label, i, 0)

            minus_btn = QPushButton(f"J{i + 1}-")
            minus_btn.setFixedSize(80, 38)
            minus_btn.setStyleSheet(self.get_btn_style("#6c757d"))
            minus_btn.clicked.connect(
                lambda checked=False, idx=i: self.on_joint_step(idx, -1.0)
            )
            joint_layout.addWidget(minus_btn, i, 1)

            plus_btn = QPushButton(f"J{i + 1}+")
            plus_btn.setFixedSize(80, 38)
            plus_btn.setStyleSheet(self.get_btn_style("#0078d4"))
            plus_btn.clicked.connect(
                lambda checked=False, idx=i: self.on_joint_step(idx, +1.0)
            )
            joint_layout.addWidget(plus_btn, i, 2)

        left_panel.addWidget(joint_group)

        # ========================================================
        # Gripper group
        # ========================================================

        gripper_group = QGroupBox("ЗАХВАТ")
        gripper_layout = QGridLayout(gripper_group)

        open_gripper_btn = QPushButton("ОТКРЫТЬ ЗАХВАТ")
        open_gripper_btn.setFixedHeight(42)
        open_gripper_btn.setStyleSheet(self.get_btn_style("#17a2b8"))
        open_gripper_btn.clicked.connect(self.on_open_gripper)
        gripper_layout.addWidget(open_gripper_btn, 0, 0)

        close_gripper_btn = QPushButton("ЗАКРЫТЬ ЗАХВАТ")
        close_gripper_btn.setFixedHeight(42)
        close_gripper_btn.setStyleSheet(self.get_btn_style("#ffc107"))
        close_gripper_btn.clicked.connect(self.on_close_gripper)
        gripper_layout.addWidget(close_gripper_btn, 0, 1)

        self.gripper_status_label = QLabel("Захват: ---")
        self.gripper_status_label.setStyleSheet(
            "background-color: #1a1a2e; "
            "color: #00ff88; "
            "padding: 8px; "
            "border: 1px solid #00cc66; "
            "border-radius: 4px;"
        )
        gripper_layout.addWidget(self.gripper_status_label, 1, 0, 1, 2)

        left_panel.addWidget(gripper_group)

        # ========================================================
        # Cartesian group disabled
        # ========================================================

        linear_group = QGroupBox("ЛИНЕЙНОЕ ДВИЖЕНИЕ X/Y/Z")
        linear_layout = QGridLayout(linear_group)

        disabled_note = QLabel(
            "Временно отключено.\n"
            "Следующий этап: подключение через MoveIt / MoveIt Servo."
        )
        disabled_note.setStyleSheet(
            "color: #ffaa00; "
            "background-color: #2d2d2d; "
            "padding: 8px; "
            "border-radius: 4px;"
        )
        linear_layout.addWidget(disabled_note, 0, 0, 1, 3)

        for label, row, col in [
            ("Z+", 1, 1),
            ("Z-", 2, 1),
            ("Y+", 2, 2),
            ("Y-", 2, 0),
            ("X+", 3, 1),
            ("X-", 4, 1),
        ]:
            btn = QPushButton(label)
            btn.setFixedSize(90, 50)
            btn.setEnabled(False)
            btn.setStyleSheet(self.get_disabled_btn_style())
            linear_layout.addWidget(btn, row, col)

        right_panel.addWidget(linear_group)

        angular_group = QGroupBox("УГЛОВОЙ ПОВОРОТ")
        angular_layout = QGridLayout(angular_group)

        for label, row, col in [
            ("RX+", 0, 0),
            ("RX-", 0, 1),
            ("RY+", 1, 0),
            ("RY-", 1, 1),
        ]:
            btn = QPushButton(label)
            btn.setFixedSize(90, 42)
            btn.setEnabled(False)
            btn.setStyleSheet(self.get_disabled_btn_style())
            angular_layout.addWidget(btn, row, col)

        rz_minus = QPushButton("RZ-")
        rz_minus.setFixedSize(90, 42)
        rz_minus.setStyleSheet(self.get_btn_style("#17a2b8"))
        rz_minus.clicked.connect(lambda: self.on_rz_step(-1.0))
        angular_layout.addWidget(rz_minus, 2, 0)

        rz_plus = QPushButton("RZ+")
        rz_plus.setFixedSize(90, 42)
        rz_plus.setStyleSheet(self.get_btn_style("#17a2b8"))
        rz_plus.clicked.connect(lambda: self.on_rz_step(+1.0))
        angular_layout.addWidget(rz_plus, 2, 1)

        right_panel.addWidget(angular_group)

        # ========================================================
        # Status group
        # ========================================================

        status_group = QGroupBox("СТАТУС")
        status_layout = QVBoxLayout(status_group)

        status_title = QLabel("ТЕКУЩИЕ УГЛЫ СУСТАВОВ:")
        status_title.setStyleSheet(
            "font-weight: bold; color: #0078d4; font-size: 11px;"
        )
        status_layout.addWidget(status_title)

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
        status_layout.addWidget(self.position_display)

        pose_label = QLabel("КООРДИНАТЫ TCP:")
        pose_label.setStyleSheet(
            "font-weight: bold; color: #0078d4; font-size: 11px; margin-top: 10px;"
        )
        status_layout.addWidget(pose_label)

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
            "font-size: 11px;"
        )
        status_layout.addWidget(self.pose_display)

        speed_label = QLabel("ШАГ СУСТАВОВ:")
        speed_label.setStyleSheet(
            "font-weight: bold; color: #ff6600; font-size: 11px; margin-top: 10px;"
        )
        status_layout.addWidget(speed_label)

        self.speed_combo = QComboBox()
        for level in self.speed_levels:
            self.speed_combo.addItem(level["label"])
        self.speed_combo.setCurrentIndex(self.speed_idx)
        self.speed_combo.currentIndexChanged.connect(self.on_speed_changed)
        self.speed_combo.setStyleSheet(
            "QComboBox { "
            " padding: 5px; "
            " border-radius: 4px; "
            " background-color: #333; "
            " color: white; "
            " border: 1px solid #555; "
            "} "
            "QComboBox::drop-down { border: none; } "
            "QComboBox::down-arrow { image: none; }"
        )
        status_layout.addWidget(self.speed_combo)

        home_btn = QPushButton("ИСХОДНАЯ ПОЗИЦИЯ")
        home_btn.setFixedHeight(42)
        home_btn.setStyleSheet(self.get_btn_style("#28a745"))
        home_btn.clicked.connect(self.on_home)
        status_layout.addWidget(home_btn)

        status_layout.addStretch()
        right_panel.addWidget(status_group)

        left_group = QGroupBox()
        left_group.setLayout(left_panel)

        right_group = QGroupBox()
        right_group.setLayout(right_panel)

        content.addWidget(left_group)
        content.addWidget(right_group)

        main_layout.addLayout(content)
        main_layout.addStretch()
        self.setLayout(main_layout)

    # ============================================================
    # Styles
    # ============================================================

    def get_btn_style(self, color: str) -> str:
        return f"""
        QPushButton {{
            background-color: {color};
            color: white;
            border: 2px solid #005a9e;
            border-radius: 8px;
            font-weight: bold;
            font-size: 10px;
            padding: 5px;
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

    def get_disabled_btn_style(self) -> str:
        return """
        QPushButton {
            background-color: #444444;
            color: #999999;
            border: 2px solid #555555;
            border-radius: 8px;
            font-weight: bold;
            font-size: 10px;
            padding: 5px;
        }
        """

    # ============================================================
    # Button handlers
    # ============================================================

    def on_speed_changed(self, index: int):
        self.speed_idx = index
        self.joint_step = self.speed_levels[self.speed_idx]["joint_step"]

        if hasattr(self.robot, "speed_scale"):
            self.robot.speed_scale = self.speed_levels[self.speed_idx]["scale"]

        self.speed_combo.blockSignals(True)
        self.speed_combo.setCurrentIndex(index)
        self.speed_combo.blockSignals(False)

    def on_start_simulation(self):
        if self.robot is not None and hasattr(self.robot, "start_simulation"):
            ok = self.robot.start_simulation()
            if ok:
                self.sim_status_label.setText("Статус: Gazebo запускается / работает")
            else:
                self.sim_status_label.setText("Статус: ошибка запуска Gazebo")

    def on_stop_simulation(self):
        if self.robot is not None and hasattr(self.robot, "stop_simulation"):
            ok = self.robot.stop_simulation()
            if ok:
                self.sim_status_label.setText("Статус: Gazebo остановлен")
            else:
                self.sim_status_label.setText("Статус: ошибка остановки Gazebo")

    def on_open_gripper(self):
        if self.robot is not None and hasattr(self.robot, "open_gripper"):
            self.robot.open_gripper()

    def on_close_gripper(self):
        if self.robot is not None and hasattr(self.robot, "close_gripper"):
            self.robot.close_gripper()

    def on_home(self):
        if self.robot is not None:
            self.robot.reset_position()

    def on_joint_step(self, joint_index: int, direction: float):
        if self.robot is None:
            return

        current = self.robot.get_current_position()
        if len(current) < 6:
            return

        target_angle = current[joint_index] + direction * self.joint_step
        self.robot.move_joint(joint_index, target_angle, duration_sec=0.5)

    def on_rz_step(self, direction: float):
        if self.robot is None:
            return

        if hasattr(self.robot, "rotate_end_effector_world"):
            self.robot.rotate_end_effector_world(
                direction * self.angle_step,
                duration=0.5,
            )

    # ============================================================
    # Old timer handlers: kept but inactive
    # ============================================================

    def start_move(self, dx: float, dy: float, dz: float):
        self.current_move = (0.0, 0.0, 0.0)

    def stop_move(self):
        self.current_move = (0.0, 0.0, 0.0)
        self.move_timer.stop()

    def start_rotate(self, drx: float, dry: float, drz: float):
        self.current_drot = (0.0, 0.0, 0.0)

    def stop_rotate(self):
        self.current_drot = (0.0, 0.0, 0.0)
        self.rotate_timer.stop()

    def on_move_timer(self):
        return

    def on_rotate_timer(self):
        return

    # ============================================================
    # ROS update
    # ============================================================

    def on_ros_update(self, data: Dict):
        if "position" in data:
            positions = data["position"]
            angles_deg = [p * (180.0 / math.pi) for p in positions]

            status_joints = ""

            for i, angle in enumerate(angles_deg, 1):
                status_joints += f"J{i}: {angle:6.1f}° "
                if i % 3 == 0:
                    status_joints += "\n"

            self.position_display.setText(status_joints.strip())

        if "pose" in data:
            pose = data["pose"]

            self.pose_display.setText(
                f"X: {pose['x'] * 1000:7.1f} mm "
                f"Y: {pose['y'] * 1000:7.1f} mm "
                f"Z: {pose['z'] * 1000:7.1f} mm\n"
                f"RX: {pose['rx']:7.1f}° "
                f"RY: {pose['ry']:7.1f}° "
                f"RZ: {pose['rz']:7.1f}°"
            )

        if "gripper_position" in data:
            gripper = data["gripper_position"]

            if len(gripper) >= 2:
                self.gripper_status_label.setText(
                    f"Захват: L={gripper[0] * 1000:.1f} мм, "
                    f"R={gripper[1] * 1000:.1f} мм"
                )
            elif len(gripper) == 1:
                self.gripper_status_label.setText(
                    f"Захват: {gripper[0] * 1000:.1f} мм"
                )

        if "simulation_running" in data:
            if data["simulation_running"]:
                self.sim_status_label.setText("Статус: Gazebo запущен из GUI")