import time
from pathlib import Path
from typing import List, Optional

from PyQt6.QtCore import QObject, QThread, Qt, pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from robot_arm_controller.core.program_executor import (
    ProgramCommand,
    ProgramParseError,
    ProgramStopped,
    RobotProgramExecutor,
    RobotProgramParser,
    RobotProgramStorage,
)


class ProgramWorker(QObject):
    log_signal = pyqtSignal(str, str)
    progress_signal = pyqtSignal(int, int, str)

    # status:
    #   "success"
    #   "stopped"
    #   "error"
    finished_signal = pyqtSignal(str, str)

    def __init__(self, executor: RobotProgramExecutor, commands: List[ProgramCommand]):
        super().__init__()
        self.executor = executor
        self.commands = commands

    def run(self) -> None:
        try:
            self.executor.run_commands(self.commands)
            self.finished_signal.emit("success", "Программа завершена успешно.")
        except ProgramStopped as exc:
            self.finished_signal.emit("stopped", str(exc))
        except Exception as exc:
            self.finished_signal.emit("error", str(exc))


class ProgramEditorUI(QWidget):
    """
    Editor and runner for .robot programs.

    UI responsibilities:
    - edit .robot source;
    - validate source;
    - save/load/delete programs;
    - run program in a worker thread;
    - show execution status and logs.

    Core logic is implemented in core/program_executor.py.
    """

    def __init__(self, robot_controller):
        super().__init__()

        self.robot = robot_controller

        package_root = Path(__file__).resolve().parents[1]
        self.programs_dir = package_root / "programs"

        self.parser = RobotProgramParser()
        self.storage = RobotProgramStorage(self.programs_dir)

        self.executor: Optional[RobotProgramExecutor] = None
        self.worker_thread: Optional[QThread] = None
        self.worker: Optional[ProgramWorker] = None
        self.is_running = False
        self.stop_requested = False

        self.init_ui()

        self.setMinimumSize(760, 520)

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def init_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 8, 10, 8)
        main_layout.setSpacing(8)

        title = QLabel("РЕДАКТОР .ROBOT-ПРОГРАММ")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setStyleSheet("color: #0078d4; margin-bottom: 4px;")
        main_layout.addWidget(title)

        self.main_splitter = QSplitter(Qt.Orientation.Horizontal)

        self.left_panel = self.build_left_panel()
        self.right_panel = self.build_right_panel()

        self.main_splitter.addWidget(self.left_panel)
        self.main_splitter.addWidget(self.right_panel)

        self.main_splitter.setStretchFactor(0, 0)
        self.main_splitter.setStretchFactor(1, 1)
        self.main_splitter.setSizes([330, 1000])

        main_layout.addWidget(self.main_splitter, 1)

        self.setLayout(main_layout)

    def build_left_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 6, 0)
        layout.setSpacing(8)

        layout.addWidget(self.build_file_panel())
        layout.addWidget(self.build_help_panel(), 1)

        return panel

    def build_right_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(6, 0, 0, 0)
        layout.setSpacing(8)

        self.vertical_splitter = QSplitter(Qt.Orientation.Vertical)

        editor_block = self.build_editor_panel()
        bottom_block = self.build_bottom_panel()

        self.vertical_splitter.addWidget(editor_block)
        self.vertical_splitter.addWidget(bottom_block)

        self.vertical_splitter.setStretchFactor(0, 3)
        self.vertical_splitter.setStretchFactor(1, 1)
        self.vertical_splitter.setSizes([560, 260])

        layout.addWidget(self.vertical_splitter, 1)

        return panel

    def build_file_panel(self) -> QGroupBox:
        group = QGroupBox("ФАЙЛ ПРОГРАММЫ")
        layout = QVBoxLayout(group)
        layout.setSpacing(8)

        name_label = QLabel("Имя программы:")
        layout.addWidget(name_label)

        self.program_name_input = QLineEdit()
        self.program_name_input.setPlaceholderText("example_program")
        layout.addWidget(self.program_name_input)

        buttons_row_1 = QHBoxLayout()

        new_btn = QPushButton("Новая")
        new_btn.clicked.connect(self.on_new_program)
        buttons_row_1.addWidget(new_btn)

        open_btn = QPushButton("Открыть")
        open_btn.clicked.connect(self.on_open_program_dialog)
        buttons_row_1.addWidget(open_btn)

        layout.addLayout(buttons_row_1)

        buttons_row_2 = QHBoxLayout()

        save_btn = QPushButton("Сохранить")
        save_btn.clicked.connect(self.on_save_program)
        save_btn.setStyleSheet(self.green_button_style())
        buttons_row_2.addWidget(save_btn)

        delete_btn = QPushButton("Удалить")
        delete_btn.clicked.connect(self.on_delete_program)
        delete_btn.setStyleSheet(self.red_button_style())
        buttons_row_2.addWidget(delete_btn)

        layout.addLayout(buttons_row_2)

        return group

    def build_help_panel(self) -> QGroupBox:
        group = QGroupBox("СПРАВКА ПО КОМАНДАМ")
        layout = QVBoxLayout(group)

        self.help_display = QPlainTextEdit()
        self.help_display.setReadOnly(True)
        self.help_display.setPlainText(
            "Основные команды:\n"
            "\n"
            "reset_home()\n"
            "reset_home(duration)\n"
            "\n"
            "move_lin(dx, dy, dz, duration)\n"
            "  dx, dy, dz — метры\n"
            "  duration — секунды\n"
            "\n"
            "rotate_rx(angle_deg, duration)\n"
            "rotate_ry(angle_deg, duration)\n"
            "rotate_rz(angle_deg, duration)\n"
            "  angle_deg — градусы\n"
            "\n"
            "joint_set(index, angle_deg, duration)\n"
            "  index: 1..6\n"
            "\n"
            "wait(duration)\n"
            "\n"
            "grip_open()\n"
            "grip_open(duration)\n"
            "grip_close()\n"
            "grip_close(duration)\n"
            "grip_set(opening_m, duration)\n"
            "  opening_m: 0..0.025\n"
            "\n"
            "save_ref()\n"
            "align_to_ref(duration)\n"
            "\n"
            "Циклы:\n"
            "for i in range(3):\n"
            "    move_lin(0.01, 0, 0, 0.5)\n"
            "\n"
            "Переменная цикла:\n"
            "for i in range(1, 4):\n"
            "    move_lin($i, 0, 0, 0.5)\n"
        )
        self.help_display.setStyleSheet(
            "background-color: #181818; "
            "color: #dddddd; "
            "font-family: Courier; "
            "font-size: 10px; "
            "padding: 8px; "
            "border: 1px solid #333333; "
            "border-radius: 4px;"
        )
        layout.addWidget(self.help_display)

        return group

    def build_editor_panel(self) -> QGroupBox:
        group = QGroupBox("КОД ПРОГРАММЫ")
        layout = QVBoxLayout(group)
        layout.setSpacing(6)

        self.code_editor = QPlainTextEdit()
        self.code_editor.setPlaceholderText(
            "# Пример программы\n"
            "reset_home()\n"
            "wait(0.5)\n"
            "\n"
            "move_lin(0.02, 0, 0, 1.0)\n"
            "wait(0.3)\n"
            "\n"
            "rotate_rz(20, 1.0)\n"
            "wait(0.3)\n"
            "\n"
            "grip_open(0.7)\n"
            "wait(0.5)\n"
            "grip_close(0.7)\n"
        )
        self.code_editor.setStyleSheet(
            "background-color: #2d2d2d; "
            "color: #00ff00; "
            "font-family: 'Courier New', monospace; "
            "font-size: 13px; "
            "padding: 10px; "
            "border: 1px solid #444444; "
            "border-radius: 4px;"
        )
        layout.addWidget(self.code_editor, 1)

        self.validation_display = QPlainTextEdit()
        self.validation_display.setReadOnly(True)
        self.validation_display.setMaximumHeight(70)
        self.validation_display.setStyleSheet(
            "background-color: #151515; "
            "color: #dddddd; "
            "font-family: Courier; "
            "font-size: 10px; "
            "padding: 6px; "
            "border: 1px solid #333333; "
            "border-radius: 4px;"
        )
        layout.addWidget(self.validation_display)

        return group

    def build_bottom_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        layout.addWidget(self.build_execution_panel())
        layout.addWidget(self.build_terminal_panel())
        layout.addWidget(self.build_log_panel(), 1)

        return panel

    def build_execution_panel(self) -> QGroupBox:
        group = QGroupBox("ВЫПОЛНЕНИЕ ПРОГРАММЫ")
        layout = QHBoxLayout(group)

        validate_btn = QPushButton("ПРОВЕРИТЬ")
        validate_btn.clicked.connect(self.on_validate_program)
        validate_btn.setStyleSheet(self.blue_button_style())
        validate_btn.setMinimumHeight(36)
        layout.addWidget(validate_btn)

        self.run_btn = QPushButton("▶ ВЫПОЛНИТЬ")
        self.run_btn.clicked.connect(self.on_run_program)
        self.run_btn.setStyleSheet(self.green_button_style())
        self.run_btn.setMinimumHeight(36)
        layout.addWidget(self.run_btn)

        self.stop_btn = QPushButton("⏹ ОСТАНОВИТЬ")
        self.stop_btn.clicked.connect(self.on_stop_program)
        self.stop_btn.setStyleSheet(self.orange_button_style())
        self.stop_btn.setMinimumHeight(36)
        self.stop_btn.setEnabled(False)
        layout.addWidget(self.stop_btn)

        self.status_label = QLabel("Статус: ожидание")
        self.status_label.setStyleSheet(
            "font-weight: bold; "
            "color: #0078d4; "
            "padding-left: 12px;"
        )
        layout.addWidget(self.status_label, 1)

        return group

    def build_terminal_panel(self) -> QGroupBox:
        group = QGroupBox("ТЕРМИНАЛ ОДНОЙ КОМАНДЫ")
        layout = QHBoxLayout(group)

        self.term_input = QLineEdit()
        self.term_input.setPlaceholderText(
            "Например: move_lin(0.01, 0, 0, 0.5)"
        )
        self.term_input.returnPressed.connect(self.on_term_execute)
        layout.addWidget(self.term_input, 1)

        run_btn = QPushButton("Выполнить")
        run_btn.clicked.connect(self.on_term_execute)
        run_btn.setStyleSheet(self.blue_button_style())
        layout.addWidget(run_btn)

        return group

    def build_log_panel(self) -> QGroupBox:
        group = QGroupBox("ЛОГ ВЫПОЛНЕНИЯ")
        layout = QVBoxLayout(group)

        self.log_display = QPlainTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setMaximumBlockCount(500)
        self.log_display.setStyleSheet(
            "background-color: #111111; "
            "color: #d7ffd7; "
            "font-family: Courier; "
            "font-size: 10px; "
            "padding: 8px; "
            "border: 1px solid #333333; "
            "border-radius: 4px;"
        )
        layout.addWidget(self.log_display, 1)

        clear_btn = QPushButton("Очистить лог")
        clear_btn.clicked.connect(self.log_display.clear)
        clear_btn.setStyleSheet(self.blue_button_style())
        layout.addWidget(clear_btn)

        return group

    # ------------------------------------------------------------------
    # Styles
    # ------------------------------------------------------------------

    @staticmethod
    def green_button_style() -> str:
        return (
            "QPushButton { "
            "background-color: #28a745; "
            "color: white; "
            "font-weight: bold; "
            "border-radius: 6px; "
            "padding: 8px; "
            "} "
            "QPushButton:disabled { "
            "background-color: #666666; "
            "color: #bbbbbb; "
            "}"
        )

    @staticmethod
    def red_button_style() -> str:
        return (
            "QPushButton { "
            "background-color: #dc3545; "
            "color: white; "
            "font-weight: bold; "
            "border-radius: 6px; "
            "padding: 8px; "
            "} "
            "QPushButton:disabled { "
            "background-color: #666666; "
            "color: #bbbbbb; "
            "}"
        )

    @staticmethod
    def orange_button_style() -> str:
        return (
            "QPushButton { "
            "background-color: #fd7e14; "
            "color: white; "
            "font-weight: bold; "
            "border-radius: 6px; "
            "padding: 8px; "
            "} "
            "QPushButton:disabled { "
            "background-color: #666666; "
            "color: #bbbbbb; "
            "}"
        )

    @staticmethod
    def blue_button_style() -> str:
        return (
            "QPushButton { "
            "background-color: #0078d4; "
            "color: white; "
            "font-weight: bold; "
            "border-radius: 6px; "
            "padding: 8px; "
            "} "
            "QPushButton:disabled { "
            "background-color: #666666; "
            "color: #bbbbbb; "
            "}"
        )

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def log(self, message: str, level: str = "INFO") -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_display.appendPlainText(f"[{timestamp}] [{level}] {message}")

        scrollbar = self.log_display.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def set_status(self, message: str, color: str = "#0078d4") -> None:
        self.status_label.setText(f"Статус: {message}")
        self.status_label.setStyleSheet(
            f"font-weight: bold; "
            f"color: {color}; "
            f"padding-left: 12px;"
        )

    # ------------------------------------------------------------------
    # File operations
    # ------------------------------------------------------------------

    def on_new_program(self) -> None:
        if self.is_running:
            QMessageBox.warning(
                self,
                "Выполнение активно",
                "Нельзя создать новую программу во время выполнения.",
            )
            return

        self.program_name_input.clear()
        self.code_editor.clear()
        self.validation_display.clear()
        self.log("Создана новая программа.")

    def on_open_program_dialog(self) -> None:
        if self.is_running:
            QMessageBox.warning(
                self,
                "Выполнение активно",
                "Нельзя открыть программу во время выполнения.",
            )
            return

        programs = self.storage.list_programs()

        if not programs:
            QMessageBox.information(
                self,
                "Нет программ",
                "Папка программ пока пустая.",
            )
            return

        dialog = QDialog(self)
        dialog.setWindowTitle("Открыть программу")
        dialog.resize(400, 320)

        layout = QVBoxLayout(dialog)
        layout.addWidget(QLabel("Выберите программу:"))

        program_list = QListWidget()
        for program_name in programs:
            program_list.addItem(QListWidgetItem(program_name))

        layout.addWidget(program_list)

        buttons = QHBoxLayout()

        open_btn = QPushButton("Открыть")
        cancel_btn = QPushButton("Отмена")

        buttons.addWidget(open_btn)
        buttons.addWidget(cancel_btn)
        layout.addLayout(buttons)

        def open_selected() -> None:
            item = program_list.currentItem()

            if item is None:
                return

            self.load_program(item.text())
            dialog.accept()

        open_btn.clicked.connect(open_selected)
        cancel_btn.clicked.connect(dialog.reject)
        program_list.itemDoubleClicked.connect(lambda _: open_selected())

        dialog.exec()

    def load_program(self, program_name: str) -> None:
        try:
            source = self.storage.load_program(program_name)

            self.program_name_input.setText(program_name)
            self.code_editor.setPlainText(source)
            self.validation_display.clear()

            self.log(f"Программа загружена: {program_name}")

        except Exception as exc:
            QMessageBox.critical(
                self,
                "Ошибка загрузки",
                str(exc),
            )
            self.log(f"Ошибка загрузки: {exc}", "ERROR")

    def on_save_program(self) -> None:
        if self.is_running:
            QMessageBox.warning(
                self,
                "Выполнение активно",
                "Нельзя сохранить программу во время выполнения.",
            )
            return

        name = self.program_name_input.text().strip()
        source = self.code_editor.toPlainText()

        if not name:
            QMessageBox.warning(
                self,
                "Ошибка",
                "Введите имя программы.",
            )
            return

        if not source.strip():
            QMessageBox.warning(
                self,
                "Ошибка",
                "Программа пустая.",
            )
            return

        try:
            commands = self.parser.parse(source)
            path = self.storage.save_program(name, source, commands)

            self.validation_display.setPlainText(
                f"OK: программа валидна. Команд: {len(commands)}"
            )
            self.log(f"Программа сохранена: {path}")

            QMessageBox.information(
                self,
                "Сохранено",
                f"Программа сохранена:\n{path}",
            )

        except ProgramParseError as exc:
            self.validation_display.setPlainText(str(exc))
            QMessageBox.critical(
                self,
                "Ошибка синтаксиса",
                str(exc),
            )
            self.log(f"Ошибка синтаксиса: {exc}", "ERROR")

        except Exception as exc:
            QMessageBox.critical(
                self,
                "Ошибка сохранения",
                str(exc),
            )
            self.log(f"Ошибка сохранения: {exc}", "ERROR")

    def on_delete_program(self) -> None:
        if self.is_running:
            QMessageBox.warning(
                self,
                "Выполнение активно",
                "Нельзя удалить программу во время выполнения.",
            )
            return

        name = self.program_name_input.text().strip()

        if not name:
            QMessageBox.warning(
                self,
                "Ошибка",
                "Введите имя программы.",
            )
            return

        reply = QMessageBox.question(
            self,
            "Удалить программу?",
            f"Удалить программу '{name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )

        if reply != QMessageBox.StandardButton.Yes:
            return

        try:
            path = self.storage.delete_program(name)

            self.program_name_input.clear()
            self.code_editor.clear()
            self.validation_display.clear()

            self.log(f"Программа удалена: {path}")

        except Exception as exc:
            QMessageBox.critical(
                self,
                "Ошибка удаления",
                str(exc),
            )
            self.log(f"Ошибка удаления: {exc}", "ERROR")

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------

    def on_validate_program(self) -> bool:
        source = self.code_editor.toPlainText()

        ok, messages, commands = self.parser.validate_source(source)

        self.validation_display.setPlainText("\n".join(messages))

        if ok:
            self.log(f"Проверка успешна. Команд: {len(commands)}")
        else:
            self.log("Проверка не пройдена.", "ERROR")

        return ok

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    def on_run_program(self) -> None:
        if self.is_running:
            QMessageBox.warning(
                self,
                "Выполнение уже идёт",
                "Сначала остановите текущую программу.",
            )
            return

        source = self.code_editor.toPlainText()

        if not source.strip():
            QMessageBox.warning(
                self,
                "Ошибка",
                "Программа пустая.",
            )
            return

        try:
            commands = self.parser.parse(source)
        except ProgramParseError as exc:
            self.validation_display.setPlainText(str(exc))
            QMessageBox.critical(
                self,
                "Ошибка синтаксиса",
                str(exc),
            )
            self.log(f"Ошибка синтаксиса: {exc}", "ERROR")
            return

        if not commands:
            QMessageBox.warning(
                self,
                "Нет команд",
                "Программа не содержит команд.",
            )
            return

        self.log_display.clear()
        self.validation_display.setPlainText(
            f"OK: программа валидна. Команд: {len(commands)}"
        )

        self.start_worker(commands)

    def on_term_execute(self) -> None:
        if self.is_running:
            QMessageBox.warning(
                self,
                "Выполнение активно",
                "Нельзя выполнить терминальную команду во время выполнения программы.",
            )
            return

        command_text = self.term_input.text().strip()

        if not command_text:
            return

        try:
            commands = self.parser.parse(command_text)
        except ProgramParseError as exc:
            self.log(f"Ошибка команды: {exc}", "ERROR")
            return

        self.term_input.clear()
        self.start_worker(commands)

    def start_worker(self, commands: List[ProgramCommand]) -> None:
        self.is_running = True
        self.stop_requested = False

        self.run_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.set_status("выполнение", "#28a745")

        self.executor = RobotProgramExecutor(
            robot=self.robot,
            log_callback=self.on_executor_log,
            progress_callback=self.on_executor_progress,
        )

        self.worker_thread = QThread(self)
        self.worker = ProgramWorker(self.executor, commands)
        self.worker.moveToThread(self.worker_thread)

        self.worker_thread.started.connect(self.worker.run)

        self.worker.log_signal.connect(self.log)
        self.worker.progress_signal.connect(self.on_worker_progress)
        self.worker.finished_signal.connect(self.on_worker_finished)

        self.worker.finished_signal.connect(self.worker_thread.quit)
        self.worker.finished_signal.connect(self.worker.deleteLater)
        self.worker_thread.finished.connect(self.worker_thread.deleteLater)

        self.worker_thread.start()

    def on_stop_program(self) -> None:
        if not self.is_running:
            return

        self.stop_requested = True

        self.log("Остановка программы пользователем.", "WARN")
        self.set_status("остановка...", "#fd7e14")

        self.stop_btn.setEnabled(False)

        if self.executor is not None:
            self.executor.request_stop()

        try:
            if self.robot is not None and hasattr(self.robot, "stop_motion"):
                self.robot.stop_motion()
        except Exception as exc:
            self.log(f"Ошибка stop_motion: {exc}", "ERROR")

    def on_executor_log(self, message: str, level: str = "INFO") -> None:
        if self.worker is not None:
            self.worker.log_signal.emit(message, level)

    def on_executor_progress(
        self,
        index: int,
        total: int,
        command: ProgramCommand,
    ) -> None:
        if self.worker is not None:
            self.worker.progress_signal.emit(index, total, command.raw)

    def on_worker_progress(self, index: int, total: int, raw: str) -> None:
        self.set_status(f"команда {index}/{total}: {raw}", "#28a745")

    def on_worker_finished(self, status: str, message: str) -> None:
        self.is_running = False

        self.run_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

        if status == "success":
            self.set_status("завершено", "#28a745")
            self.log(message, "INFO")

        elif status == "stopped":
            self.set_status("остановлено", "#fd7e14")
            self.log("Программа остановлена пользователем.", "WARN")

        else:
            if self.stop_requested:
                self.set_status("остановлено", "#fd7e14")
                self.log("Программа остановлена пользователем.", "WARN")
            else:
                self.set_status("ошибка", "#dc3545")
                self.log(message, "ERROR")

        self.stop_requested = False
        self.executor = None
        self.worker = None
        self.worker_thread = None