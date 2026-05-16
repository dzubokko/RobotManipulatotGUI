import sys

from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QAction
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QMenu,
    QMessageBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

import rclpy

from robot_arm_controller.robot_bridge import RobotBridge
from robot_arm_controller.ui.manual_control_ui import ManualControlUI
from robot_arm_controller.ui.program_editor_ui import ProgramEditorUI
from robot_arm_controller.ui.program_manager_ui import ProgramManagerUI


class RobotControllerApp(QMainWindow):
    """
    Главное окно приложения управления роботом-манипулятором.

    Окно открывается обычным размером, чтобы работало системное
    позиционирование окна, например Win + ← / Win + →.
    """

    DEFAULT_WINDOW_WIDTH = 1180
    DEFAULT_WINDOW_HEIGHT = 760

    MIN_WINDOW_WIDTH = 760
    MIN_WINDOW_HEIGHT = 520

    def __init__(self) -> None:
        super().__init__()

        self.setWindowTitle("Контроллер робота-манипулятора + Gazebo")
        self.setMinimumSize(self.MIN_WINDOW_WIDTH, self.MIN_WINDOW_HEIGHT)
        self.setStyleSheet(self.get_stylesheet())

        self.robot_bridge: RobotBridge | None = None

        self.init_ros_bridge()
        self.init_ui()
        self.init_menu()
        self.setup_window_geometry()

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(1000)

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def init_ros_bridge(self) -> None:
        try:
            if not rclpy.ok():
                rclpy.init()

            self.robot_bridge = RobotBridge()

        except Exception as exc:
            QMessageBox.critical(
                self,
                "Ошибка ROS 2",
                "Не удалось подключиться к ROS 2:\n"
                f"{exc}\n\n"
                "Проверь:\n"
                "1) source ~/RobotManipulator/install/setup.bash\n"
                "2) запущен Gazebo:\n"
                "   ros2 launch robot_gazebo gazebo.launch.py\n"
                "3) контроллеры active:\n"
                "   ros2 control list_controllers",
            )
            self.robot_bridge = None

    def init_ui(self) -> None:
        central_widget = QWidget(self)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(6)

        self.tabs = QTabWidget(central_widget)
        self.tabs.setDocumentMode(True)
        self.tabs.setMovable(False)

        self.manual_ui = (
            ManualControlUI(self.robot_bridge)
            if self.robot_bridge is not None
            else None
        )

        self.editor_ui = (
            ProgramEditorUI(self.robot_bridge)
            if self.robot_bridge is not None
            else None
        )

        self.manager_ui = ProgramManagerUI(self.robot_bridge)

        if self.manual_ui is not None:
            self.tabs.addTab(self.manual_ui, "Ручное управление")

        if self.editor_ui is not None:
            self.tabs.addTab(self.editor_ui, "Редактор программы")

        self.tabs.addTab(self.manager_ui, "Менеджер программ")

        if self.manager_ui is not None and self.editor_ui is not None:
            self.manager_ui.program_open_requested.connect(
                self.open_program_in_editor
            )

        main_layout.addWidget(self.tabs)
        central_widget.setLayout(main_layout)

        self.setCentralWidget(central_widget)
        self.statusBar().showMessage("Приложение готово")

    def init_menu(self) -> None:
        menu_bar = self.menuBar()

        window_menu: QMenu = menu_bar.addMenu("Окно")

        reset_size_action = QAction("Открыть обычным размером", self)
        reset_size_action.triggered.connect(self.reset_window_size)
        window_menu.addAction(reset_size_action)

        toggle_maximized_action = QAction("Развернуть / восстановить", self)
        toggle_maximized_action.triggered.connect(self.toggle_maximized)
        window_menu.addAction(toggle_maximized_action)

        window_menu.addSeparator()

        exit_action = QAction("Выход", self)
        exit_action.triggered.connect(self.close)
        window_menu.addAction(exit_action)

    def setup_window_geometry(self) -> None:
        """
        Открывает окно обычным размером и центрирует его.
        Здесь не используется showMaximized() и fullscreen.
        """
        screen = QApplication.primaryScreen()

        if screen is None:
            self.resize(self.DEFAULT_WINDOW_WIDTH, self.DEFAULT_WINDOW_HEIGHT)
            return

        available = screen.availableGeometry()

        width = min(self.DEFAULT_WINDOW_WIDTH, available.width() - 120)
        height = min(self.DEFAULT_WINDOW_HEIGHT, available.height() - 120)

        width = max(self.MIN_WINDOW_WIDTH, width)
        height = max(self.MIN_WINDOW_HEIGHT, height)

        self.resize(width, height)

        x = available.x() + (available.width() - width) // 2
        y = available.y() + (available.height() - height) // 2

        self.move(x, y)

    # ------------------------------------------------------------------
    # Window actions
    # ------------------------------------------------------------------

    def reset_window_size(self) -> None:
        self.showNormal()
        self.setup_window_geometry()
        self.statusBar().showMessage("Окно открыто обычным размером", 3000)

    def toggle_maximized(self) -> None:
        if self.isMaximized():
            self.showNormal()
            self.setup_window_geometry()
            self.statusBar().showMessage("Окно восстановлено", 3000)
        else:
            self.showMaximized()
            self.statusBar().showMessage("Окно развёрнуто", 3000)

    def open_program_in_editor(self, program_name: str) -> None:
        if self.editor_ui is None:
            return

        try:
            self.editor_ui.load_program(program_name)
            self.tabs.setCurrentWidget(self.editor_ui)
            self.statusBar().showMessage(
                f"Программа открыта в редакторе: {program_name}",
                3000,
            )

        except Exception as exc:
            QMessageBox.critical(
                self,
                "Ошибка открытия программы",
                f"Не удалось открыть программу в редакторе:\n{exc}",
            )

    # ------------------------------------------------------------------
    # Styles
    # ------------------------------------------------------------------

    @staticmethod
    def get_stylesheet() -> str:
        return """
        QMainWindow {
            background-color: #1e1e1e;
            color: #ffffff;
        }

        QMenuBar {
            background-color: #1e1e1e;
            color: #ffffff;
            border-bottom: 1px solid #3d3d3d;
        }

        QMenuBar::item {
            background-color: transparent;
            padding: 5px 10px;
        }

        QMenuBar::item:selected {
            background-color: #0078d4;
        }

        QMenu {
            background-color: #2d2d2d;
            color: #ffffff;
            border: 1px solid #3d3d3d;
        }

        QMenu::item {
            padding: 6px 24px;
        }

        QMenu::item:selected {
            background-color: #0078d4;
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

        QPushButton:disabled {
            background-color: #666666;
            color: #bbbbbb;
        }

        QLineEdit,
        QTextEdit,
        QPlainTextEdit {
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

        QHeaderView::section {
            background-color: #0078d4;
            color: #ffffff;
            padding: 5px;
            border: none;
            font-weight: bold;
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

        QStatusBar {
            background-color: #1e1e1e;
            color: #ffffff;
            border-top: 1px solid #3d3d3d;
        }
        """

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def update_status(self) -> None:
        if self.robot_bridge is not None and self.robot_bridge.is_connected:
            pos = self.robot_bridge.get_current_position()

            if pos:
                j1_deg = pos[0] * 57.2957795
                self.statusBar().showMessage(
                    f"Соединение: OK | J1 = {j1_deg:.1f}°"
                )
        else:
            self.statusBar().showMessage("Соединение: нет подключения к RobotBridge")

    # ------------------------------------------------------------------
    # Closing
    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:
        reply = QMessageBox.question(
            self,
            "Выход",
            "Вы уверены, что хотите закрыть приложение?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )

        if reply == QMessageBox.StandardButton.Yes:
            if self.robot_bridge is not None:
                self.robot_bridge.shutdown()

            if rclpy.ok():
                rclpy.shutdown()

            event.accept()
        else:
            event.ignore()


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("Robot Manipulator Controller")

    window = RobotControllerApp()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()