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
    DEFAULT_WINDOW_WIDTH = 1180
    DEFAULT_WINDOW_HEIGHT = 760

    MIN_WINDOW_WIDTH = 760
    MIN_WINDOW_HEIGHT = 520

    COLOR_BG = "#171717"
    COLOR_PANEL = "#202020"
    COLOR_PANEL_2 = "#252525"
    COLOR_BORDER = "#3a3a3a"
    COLOR_TEXT = "#f2f2f2"
    COLOR_MUTED = "#a9a9a9"
    COLOR_ACCENT = "#0a84ff"
    COLOR_ACCENT_HOVER = "#1e90ff"

    def __init__(self) -> None:
        super().__init__()

        self.setWindowTitle("Контроллер робота-манипулятора")
        self.setMinimumSize(self.MIN_WINDOW_WIDTH, self.MIN_WINDOW_HEIGHT)
        self.setStyleSheet(self.get_stylesheet())

        self.robot_bridge: RobotBridge | None = None

        self.manual_ui: ManualControlUI | None = None
        self.editor_ui: ProgramEditorUI | None = None
        self.manager_ui: ProgramManagerUI | None = None

        self.init_ros_bridge()
        self.init_ui()
        self.init_menu()
        self.setup_window_geometry()

        self.status_timer = QTimer(self)
        self.status_timer.setInterval(1000)
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start()

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
                "1) cd ~/RobotManipulator\n"
                "2) source /opt/ros/humble/setup.bash\n"
                "3) source install/setup.bash",
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

        if self.robot_bridge is not None:
            self.manual_ui = ManualControlUI(self.robot_bridge)
            self.editor_ui = ProgramEditorUI(self.robot_bridge)
        else:
            self.manual_ui = None
            self.editor_ui = None

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

        main_layout.addWidget(self.tabs, 1)
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
            QMessageBox.warning(
                self,
                "Редактор недоступен",
                "Редактор программ недоступен, потому что RobotBridge не подключён.",
            )
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

    @classmethod
    def get_stylesheet(cls) -> str:
        return f"""
        QMainWindow {{
            background-color: {cls.COLOR_BG};
            color: {cls.COLOR_TEXT};
        }}

        QMenuBar {{
            background-color: {cls.COLOR_BG};
            color: {cls.COLOR_TEXT};
            border-bottom: 1px solid {cls.COLOR_BORDER};
        }}

        QMenuBar::item {{
            background-color: transparent;
            padding: 5px 10px;
        }}

        QMenuBar::item:selected {{
            background-color: {cls.COLOR_ACCENT};
        }}

        QMenu {{
            background-color: {cls.COLOR_PANEL};
            color: {cls.COLOR_TEXT};
            border: 1px solid {cls.COLOR_BORDER};
        }}

        QMenu::item {{
            padding: 6px 24px;
        }}

        QMenu::item:selected {{
            background-color: {cls.COLOR_ACCENT};
        }}

        QTabWidget::pane {{
            border: 1px solid {cls.COLOR_BORDER};
            background-color: {cls.COLOR_BG};
        }}

        QTabBar::tab {{
            background-color: {cls.COLOR_PANEL};
            color: {cls.COLOR_TEXT};
            padding: 10px 20px;
            margin: 2px;
            border: 1px solid {cls.COLOR_BORDER};
            border-radius: 8px;
            font-weight: bold;
        }}

        QTabBar::tab:selected {{
            background-color: {cls.COLOR_ACCENT};
            border: 1px solid {cls.COLOR_ACCENT};
        }}

        QTabBar::tab:hover {{
            background-color: {cls.COLOR_ACCENT_HOVER};
        }}

        QPushButton {{
            background-color: {cls.COLOR_ACCENT};
            color: {cls.COLOR_TEXT};
            border: 1px solid {cls.COLOR_ACCENT};
            padding: 8px 16px;
            border-radius: 8px;
            font-weight: bold;
            font-size: 11px;
        }}

        QPushButton:hover {{
            background-color: {cls.COLOR_ACCENT_HOVER};
        }}

        QPushButton:pressed {{
            background-color: #0f0f0f;
        }}

        QPushButton:disabled {{
            background-color: #333333;
            color: #777777;
            border: 1px solid #333333;
        }}

        QLineEdit,
        QTextEdit,
        QPlainTextEdit {{
            background-color: {cls.COLOR_PANEL_2};
            color: {cls.COLOR_TEXT};
            border: 1px solid {cls.COLOR_BORDER};
            padding: 6px;
            border-radius: 8px;
        }}

        QLabel {{
            color: {cls.COLOR_TEXT};
        }}

        QGroupBox {{
            background-color: {cls.COLOR_PANEL};
            color: {cls.COLOR_TEXT};
            border: 1px solid {cls.COLOR_BORDER};
            border-radius: 10px;
            margin-top: 10px;
            padding-top: 10px;
            font-weight: bold;
        }}

        QGroupBox::title {{
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px;
            color: {cls.COLOR_MUTED};
        }}

        QTableWidget {{
            background-color: {cls.COLOR_PANEL_2};
            color: {cls.COLOR_TEXT};
            border: 1px solid {cls.COLOR_BORDER};
            gridline-color: {cls.COLOR_BORDER};
            selection-background-color: {cls.COLOR_ACCENT};
        }}

        QHeaderView::section {{
            background-color: {cls.COLOR_PANEL};
            color: {cls.COLOR_TEXT};
            padding: 6px;
            border: none;
            border-bottom: 1px solid {cls.COLOR_BORDER};
            font-weight: bold;
        }}

        QStatusBar {{
            background-color: {cls.COLOR_BG};
            color: {cls.COLOR_TEXT};
            border-top: 1px solid {cls.COLOR_BORDER};
        }}
        """

    def update_status(self) -> None:
        if self.robot_bridge is not None and self.robot_bridge.is_connected:
            position = self.robot_bridge.get_current_position()

            if position:
                j1_deg = position[0] * 57.2957795
                self.statusBar().showMessage(
                    f"Соединение: OK | J1 = {j1_deg:.1f}°"
                )
                return

            self.statusBar().showMessage("Соединение: OK")
            return

        self.statusBar().showMessage("Соединение: нет подключения к RobotBridge")

    def closeEvent(self, event) -> None:
        reply = QMessageBox.question(
            self,
            "Выход",
            "Вы уверены, что хотите закрыть приложение?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )

        if reply != QMessageBox.StandardButton.Yes:
            event.ignore()
            return

        try:
            if self.manual_ui is not None:
                self.manual_ui.stop_gazebo_process()
        except Exception:
            pass

        try:
            if self.editor_ui is not None and getattr(self.editor_ui, "is_running", False):
                self.editor_ui.on_stop_program()
        except Exception:
            pass

        try:
            if self.robot_bridge is not None:
                self.robot_bridge.shutdown()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        event.accept()


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("Robot Manipulator Controller")

    window = RobotControllerApp()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()