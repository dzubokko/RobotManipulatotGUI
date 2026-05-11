from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QGridLayout,
    QComboBox,
    QApplication,
)
from PyQt6.QtCore import QTimer, QProcess
from PyQt6.QtGui import QFont

from pathlib import Path
from typing import Dict

import math
import os
import signal
import subprocess


class ManualControlUI(QWidget):
    def __init__(self, robot_bridge):
        super().__init__()

        self.robot = robot_bridge

        # Процесс Gazebo, запущенный из приложения
        self.gazebo_process = None
        self.gazebo_pid = None

        self.speed_levels = [
            {"label": "100 мм/с", "scale": 10.0},
            {"label": "50 мм/с", "scale": 5.0},
            {"label": "10 мм/с", "scale": 1.0},
        ]

        self.speed_idx = 1
        self.update_speed()

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
        self.robot.register_callback(self.on_ros_update)

        app = QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self.stop_gazebo_process)

    # ------------------------------------------------------------------
    # Интерфейс
    # ------------------------------------------------------------------

    def update_speed(self):
        self.robot.speed_scale = self.speed_levels[self.speed_idx]["scale"]

    def init_ui(self):
        main_layout = QVBoxLayout(self)

        title = QLabel("РУЧНОЕ УПРАВЛЕНИЕ")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setStyleSheet("color: #0078d4; margin-bottom: 15px;")
        main_layout.addWidget(title)

        content = QHBoxLayout()

        # ------------------------------------------------------------------
        # Левая панель: движение
        # ------------------------------------------------------------------

        left_panel = QVBoxLayout()

        linear_group = QGroupBox("ЛИНЕЙНОЕ ДВИЖЕНИЕ")
        linear_layout = QGridLayout(linear_group)

        up_btn = QPushButton("Z+")
        up_btn.setFixedSize(90, 60)
        up_btn.setStyleSheet(self.get_btn_style("#0078d4"))
        up_btn.pressed.connect(lambda: self.start_move(0.0, 0.0, +1.0))
        up_btn.released.connect(self.stop_move)
        linear_layout.addWidget(up_btn, 0, 1)

        down_btn = QPushButton("Z-")
        down_btn.setFixedSize(90, 60)
        down_btn.setStyleSheet(self.get_btn_style("#0078d4"))
        down_btn.pressed.connect(lambda: self.start_move(0.0, 0.0, -1.0))
        down_btn.released.connect(self.stop_move)
        linear_layout.addWidget(down_btn, 1, 1)

        left_btn = QPushButton("Y+")
        left_btn.setFixedSize(90, 60)
        left_btn.setStyleSheet(self.get_btn_style("#28a745"))
        left_btn.pressed.connect(lambda: self.start_move(0.0, -1.0, 0.0))
        left_btn.released.connect(self.stop_move)
        linear_layout.addWidget(left_btn, 1, 2)

        right_btn = QPushButton("Y-")
        right_btn.setFixedSize(90, 60)
        right_btn.setStyleSheet(self.get_btn_style("#28a745"))
        right_btn.pressed.connect(lambda: self.start_move(0.0, +1.0, 0.0))
        right_btn.released.connect(self.stop_move)
        linear_layout.addWidget(right_btn, 1, 0)

        back_btn = QPushButton("X+")
        back_btn.setFixedSize(90, 60)
        back_btn.setStyleSheet(self.get_btn_style("#dc3545"))
        back_btn.pressed.connect(lambda: self.start_move(-1.0, 0.0, 0.0))
        back_btn.released.connect(self.stop_move)
        linear_layout.addWidget(back_btn, 2, 1)

        forward_btn = QPushButton("X-")
        forward_btn.setFixedSize(90, 60)
        forward_btn.setStyleSheet(self.get_btn_style("#dc3545"))
        forward_btn.pressed.connect(lambda: self.start_move(+1.0, 0.0, 0.0))
        forward_btn.released.connect(self.stop_move)
        linear_layout.addWidget(forward_btn, 3, 1)

        left_panel.addWidget(linear_group)

        angular_group = QGroupBox("УГЛОВОЙ ПОВОРОТ")
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

        left_panel.addWidget(angular_group)
        left_panel.addStretch()

        left_group = QGroupBox()
        left_group.setLayout(left_panel)
        content.addWidget(left_group)

        # ------------------------------------------------------------------
        # Правая панель: статус, скорость, исходная позиция, захват, симуляция
        # ------------------------------------------------------------------

        right_group = QGroupBox("СТАТУС И УПРАВЛЕНИЕ")
        right_layout = QVBoxLayout()

        status_title = QLabel("ТЕКУЩИЕ УГЛЫ СУСТАВОВ:")
        status_title.setStyleSheet(
            "font-weight: bold; color: #0078d4; font-size: 11px;"
        )
        right_layout.addWidget(status_title)

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

        pose_label = QLabel("КООРДИНАТЫ")
        pose_label.setStyleSheet(
            "font-weight: bold; color: #0078d4; font-size: 11px; margin-top: 10px;"
        )
        right_layout.addWidget(pose_label)

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
        right_layout.addWidget(self.pose_display)

        speed_label = QLabel("СКОРОСТЬ ПОДАЧИ:")
        speed_label.setStyleSheet(
            "font-weight: bold; color: #ff6600; font-size: 11px; margin-top: 10px;"
        )
        right_layout.addWidget(speed_label)

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
        right_layout.addWidget(self.speed_combo)

        # Исходная позиция
        home_layout = QHBoxLayout()

        home_btn = QPushButton("ИСХОДНАЯ ПОЗИЦИЯ")
        home_btn.setStyleSheet(self.get_btn_style("#28a745"))
        home_btn.clicked.connect(self.on_home)
        home_layout.addWidget(home_btn)

        right_layout.addLayout(home_layout)

        # Захват
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

        # Симуляция
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

        right_group.setLayout(right_layout)
        content.addWidget(right_group)

        main_layout.addLayout(content)
        main_layout.addStretch()

        self.setLayout(main_layout)

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

    # ------------------------------------------------------------------
    # Ручное управление
    # ------------------------------------------------------------------

    def on_speed_changed(self, index: int):
        self.speed_idx = index
        self.update_speed()

        self.speed_combo.blockSignals(True)
        self.speed_combo.setCurrentIndex(index)
        self.speed_combo.blockSignals(False)

    def start_move(self, dx: float, dy: float, dz: float):
        length = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5

        if length == 0:
            return

        k = 1.0 / length
        self.current_move = (dx * k, dy * k, dz * k)

        if not self.move_timer.isActive():
            self.move_timer.start()

    def stop_move(self):
        self.current_move = (0.0, 0.0, 0.0)
        self.move_timer.stop()

    def start_rotate(self, drx: float, dry: float, drz: float):
        length = (drx ** 2 + dry ** 2 + drz ** 2) ** 0.5

        if length == 0:
            return

        k = 1.0 / length
        self.current_drot = (drx * k, dry * k, drz * k)

        if not self.rotate_timer.isActive():
            self.rotate_timer.start()

    def stop_rotate(self):
        self.current_drot = (0.0, 0.0, 0.0)
        self.rotate_timer.stop()

    def on_home(self):
        self.robot.reset_position()

    def on_gripper_open(self):
        self.robot.open_gripper(0.7)

    def on_gripper_close(self):
        self.robot.close_gripper(0.7)

    def on_move_timer(self):
        dx, dy, dz = self.current_move

        if dx == dy == dz == 0.0:
            return

        base_step_m = 0.001
        scaled_step_size = base_step_m * self.robot.speed_scale

        self.robot.move_end_effector_world(
            dx * scaled_step_size,
            dy * scaled_step_size,
            dz * scaled_step_size,
            duration=0.05,
        )

    def on_rotate_timer(self):
        drx, dry, drz = self.current_drot

        if drx == dry == drz == 0.0:
            return

        angle_delta = self.angle_step

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

    # ------------------------------------------------------------------
    # Обновление данных от ROS
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Запуск и остановка Gazebo из приложения
    # ------------------------------------------------------------------

    def find_workspace_root(self) -> Path:
        """
        Ищет корень рабочего пространства RobotManipulator.

        Ожидаем структуру:
        RobotManipulator/
          src/
            robot_gazebo/
              launch/
                gazebo.launch.py
        """

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

    def on_start_gazebo(self):
        """
        Запуск Gazebo прямо из приложения без всплывающих окон.
        """

        if (
            self.gazebo_process is not None
            and self.gazebo_process.state() != QProcess.ProcessState.NotRunning
        ):
            print("[Gazebo] Симуляция уже запущена.")
            return

        workspace_root = self.find_workspace_root()
        setup_file = workspace_root / "install" / "setup.bash"

        if not setup_file.exists():
            print("[Gazebo] install/setup.bash не найден.")
            print("[Gazebo] Выполни:")
            print("cd ~/RobotManipulator")
            print("colcon build")
            print("source install/setup.bash")
            return

        command = (
            f'cd "{workspace_root}" && '
            f"source /opt/ros/humble/setup.bash && "
            f"source install/setup.bash && "
            f"ros2 launch robot_gazebo gazebo.launch.py"
        )

        self.gazebo_process = QProcess(self)

        # setsid запускает процесс в отдельной process group.
        # Это нужно, чтобы кнопка остановки закрывала не только ros2 launch,
        # но и дочерние процессы Gazebo.
        self.gazebo_process.setProgram("setsid")
        self.gazebo_process.setArguments(["bash", "-lc", command])

        self.gazebo_process.setProcessChannelMode(
            QProcess.ProcessChannelMode.MergedChannels
        )

        self.gazebo_process.readyReadStandardOutput.connect(
            self.on_gazebo_output
        )
        self.gazebo_process.finished.connect(self.on_gazebo_finished)

        self.gazebo_process.start()

        if not self.gazebo_process.waitForStarted(3000):
            print("[Gazebo] Не удалось запустить симуляцию.")
            self.gazebo_process = None
            self.gazebo_pid = None
            return

        self.gazebo_pid = int(self.gazebo_process.processId())

        print("[Gazebo] Симуляция запускается.")
        print("[Gazebo] Подожди 10–15 секунд, пока загрузятся робот и контроллеры.")

    def on_stop_gazebo(self):
        """
        Остановка Gazebo из приложения без всплывающих окон.
        """

        print("[Gazebo] Остановка симуляции...")
        self.stop_gazebo_process()

    def stop_gazebo_process(self):
        """
        Корректно завершает Gazebo/ros2 launch и дочерние процессы.
        """

        if self.gazebo_process is None:
            self.cleanup_gazebo_processes()
            print("[Gazebo] Активный процесс приложения не найден. Выполнена очистка.")
            return

        if self.gazebo_process.state() == QProcess.ProcessState.NotRunning:
            self.cleanup_gazebo_processes()
            self.gazebo_process = None
            self.gazebo_pid = None
            print("[Gazebo] Симуляция уже остановлена.")
            return

        # 1. Пробуем остановить всю process group
        if self.gazebo_pid is not None:
            try:
                os.killpg(os.getpgid(self.gazebo_pid), signal.SIGTERM)
                print("[Gazebo] SIGTERM отправлен группе процессов.")
            except ProcessLookupError:
                print("[Gazebo] Группа процессов уже завершена.")
            except Exception as exc:
                print(f"[Gazebo] Не удалось остановить группу процессов: {exc}")

        # 2. Ждём мягкого завершения
        if not self.gazebo_process.waitForFinished(5000):
            print("[Gazebo] Мягкая остановка не сработала. Принудительное завершение...")

            if self.gazebo_pid is not None:
                try:
                    os.killpg(os.getpgid(self.gazebo_pid), signal.SIGKILL)
                    print("[Gazebo] SIGKILL отправлен группе процессов.")
                except Exception as exc:
                    print(f"[Gazebo] Не удалось принудительно завершить группу: {exc}")

            self.gazebo_process.kill()
            self.gazebo_process.waitForFinished(3000)

        # 3. На всякий случай чистим оставшиеся процессы Gazebo
        self.cleanup_gazebo_processes()

        self.gazebo_process = None
        self.gazebo_pid = None

        print("[Gazebo] Симуляция остановлена.")

    def cleanup_gazebo_processes(self):
        """
        Дополнительная очистка зависших процессов Gazebo.

        Нужна потому, что ros2 launch может завершиться,
        а gzserver/gzclient иногда остаются висеть.
        """

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
            )
        except Exception as exc:
            print(f"[Gazebo] Ошибка при очистке процессов: {exc}")

    def on_gazebo_finished(self):
        """
        Срабатывает, когда процесс Gazebo завершился сам.
        """

        print("[Gazebo] Процесс симуляции завершён.")
        self.gazebo_process = None
        self.gazebo_pid = None

    def on_gazebo_output(self):
        """
        Читает вывод Gazebo, чтобы процесс не зависал из-за заполненного буфера.
        """

        if self.gazebo_process is None:
            return

        output = bytes(self.gazebo_process.readAllStandardOutput()).decode(
            errors="ignore"
        )

        text = output.strip()

        if text:
            print(text)