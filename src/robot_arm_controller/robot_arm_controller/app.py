"""
Главное приложение PyQt6 для управления UR5 в Gazebo.
"""

import sys

from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QTabWidget,
    QWidget,
    QVBoxLayout,
    QMessageBox,
)
from PyQt6.QtCore import QTimer
import rclpy

from robot_bridge import RobotBridge
from ui.manual_control_ui import ManualControlUI
from ui.program_editor_ui import ProgramEditorUI
from ui.program_manager_ui import ProgramManagerUI


class RobotControllerApp(QMainWindow):
    """Главное окно приложения управления UR5."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Контроллер UR5 + Gazebo")
        self.setGeometry(50, 50, 1600, 1000)
        self.setStyleSheet(self.get_stylesheet())

        self.robot_bridge: RobotBridge | None = None

        # Инициализируем ROS 2
        try:
            if not rclpy.ok():
                rclpy.init()
            self.robot_bridge = RobotBridge()
        except Exception as e:
            QMessageBox.critical(
                self,
                "Ошибка ROS 2",
                "Не удалось подключиться к ROS 2:\n"
                f"{e}\n\n"
                "Убедись, что:\n"
                "1) Окружение ROS 2 активировано.\n"
                "2) Запущена симуляция Gazebo:\n"
                "   ros2 launch ur_simulation_gazebo "
                "ur_sim_controller.launch.py ur_type:=ur5",
            )
            self.robot_bridge = None

        self.init_ui()

        # Таймер статуса
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(1000)

    def init_ui(self) -> None:
        """Инициализировать интерфейс."""

        central_widget = QWidget(self)
        main_layout = QVBoxLayout(central_widget)

        self.tabs = QTabWidget(central_widget)

        # Панели UI
        self.manual_ui = (
            ManualControlUI(self.robot_bridge) if self.robot_bridge else None
        )
        self.editor_ui = (
            ProgramEditorUI(self.robot_bridge) if self.robot_bridge else None
        )
        self.manager_ui = ProgramManagerUI()

        if self.manual_ui is not None:
            self.tabs.addTab(self.manual_ui, "Ручное управление")
        if self.editor_ui is not None:
            self.tabs.addTab(self.editor_ui, "Редактор программы")
        self.tabs.addTab(self.manager_ui, "Менеджер программ")

        main_layout.addWidget(self.tabs)
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        self.statusBar().showMessage("Приложение готово")

    @staticmethod
    def get_stylesheet() -> str:
        """Тёмная тема."""
        return """
            QMainWindow {
                background-color: #1e1e1e;
                color: #ffffff;
            }
            QTabWidget::pane {
                border: 2px solid #3d3d3d;
            }
            QTabBar::tab {
                background-color: #2d2d2d;
                color: #ffffff;
                padding: 10px 20px;
                margin: 2px;
                border: 1px solid #3d3d3d;
                font-weight: bold;
            }
            QTabBar::tab:selected {
                background-color: #0078d4;
                border: 1px solid #0078d4;
            }
            QPushButton {
                background-color: #0078d4;
                color: white;
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
                font-size: 11px;
            }
            QPushButton:hover {
                background-color: #1084d8;
            }
            QPushButton:pressed {
                background-color: #005a9e;
            }
            QLineEdit, QTextEdit {
                background-color: #2d2d2d;
                color: #ffffff;
                border: 1px solid #3d3d3d;
                padding: 6px;
                border-radius: 4px;
            }
            QLabel {
                color: #ffffff;
            }
            QGroupBox {
                color: #ffffff;
                border: 2px solid #3d3d3d;
                border-radius: 4px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px 0 3px;
            }
            QTableWidget {
                background-color: #2d2d2d;
                color: #ffffff;
                border: 1px solid #3d3d3d;
            }
            QSlider::groove:horizontal {
                border: 1px solid #3d3d3d;
                height: 8px;
                background: #2d2d2d;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #0078d4;
                border: 1px solid #0078d4;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
        """

    def update_status(self) -> None:
        """Обновить строку состояния."""
        if self.robot_bridge is not None and self.robot_bridge.is_connected:
            pos = self.robot_bridge.get_current_position()
            if pos:
                j1_deg = pos[0] * 57.2957795
                self.statusBar().showMessage(f"Соединение: OK, J1 = {j1_deg:.1f}°")

    def closeEvent(self, event) -> None:
        """Обработчик закрытия окна."""
        reply = QMessageBox.question(
            self,
            "Выход",
            "Вы уверены, что хотите закрыть приложение?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )

        if reply == QMessageBox.StandardButton.Yes:
            if self.robot_bridge is not None:
                self.robot_bridge.shutdown()
            event.accept()
        else:
            event.ignore()


def main() -> None:
    """Точка входа приложения."""
    app = QApplication(sys.argv)
    window = RobotControllerApp()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
