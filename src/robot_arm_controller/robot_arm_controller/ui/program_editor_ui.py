import time
from pathlib import Path
from typing import List, Optional

from PyQt6.QtCore import QObject, QThread, Qt, pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QComboBox,
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
    program_saved = pyqtSignal(str)
    program_deleted = pyqtSignal(str)

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
    COLOR_SUCCESS = "#2da44e"
    COLOR_LOG = "#101010"

    def __init__(self, robot_bridge=None):
        super().__init__()

        self.robot_bridge = robot_bridge

        package_root = Path(__file__).resolve().parents[1]
        self.programs_dir = package_root / "programs"

        self.parser = RobotProgramParser()
        self.storage = RobotProgramStorage(self.programs_dir)

        self.current_program_name: Optional[str] = None
        self.last_valid_commands: List[ProgramCommand] = []

        self.executor: Optional[RobotProgramExecutor] = None
        self.worker_thread: Optional[QThread] = None
        self.worker: Optional[ProgramWorker] = None
        self.is_running = False
        self.stop_requested = False

        self.init_ui()
        self.refresh_program_list()
        self.new_program()

    def init_ui(self) -> None:
        self.setStyleSheet(self.base_stylesheet())

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(12, 10, 12, 10)
        main_layout.setSpacing(10)

        title = QLabel("РЕДАКТОР ПРОГРАММ")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setStyleSheet(f"color: {self.COLOR_TEXT};")
        main_layout.addWidget(title)

        self.main_splitter = QSplitter(Qt.Orientation.Horizontal)

        left_panel = self.build_left_panel()
        right_panel = self.build_right_panel()

        self.main_splitter.addWidget(left_panel)
        self.main_splitter.addWidget(right_panel)
        self.main_splitter.setStretchFactor(0, 1)
        self.main_splitter.setStretchFactor(1, 3)
        self.main_splitter.setSizes([340, 900])

        main_layout.addWidget(self.main_splitter, 1)

        self.setMinimumSize(760, 520)
        self.setLayout(main_layout)

    def build_left_panel(self) -> QWidget:
        panel = QWidget()
        panel.setMinimumWidth(280)

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 6, 0)
        layout.setSpacing(10)

        layout.addWidget(self.build_file_panel())
        layout.addWidget(self.build_program_list_panel(), 1)
        layout.addWidget(self.build_help_panel(), 2)

        return panel

    def build_right_panel(self) -> QWidget:
        panel = QWidget()

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(6, 0, 0, 0)
        layout.setSpacing(10)

        layout.addWidget(self.build_editor_panel(), 4)
        layout.addWidget(self.build_execution_panel(), 2)

        return panel

    def build_file_panel(self) -> QGroupBox:
        group = QGroupBox("ФАЙЛ ПРОГРАММЫ")
        layout = QVBoxLayout(group)
        layout.setSpacing(8)

        name_row = QHBoxLayout()

        self.program_name_input = QLineEdit()
        self.program_name_input.setPlaceholderText("Имя программы")
        self.program_name_input.setStyleSheet(self.line_edit_style())
        name_row.addWidget(self.program_name_input, 1)

        layout.addLayout(name_row)

        row_1 = QHBoxLayout()

        new_btn = QPushButton("Новая")
        new_btn.clicked.connect(self.new_program)
        new_btn.setStyleSheet(self.button_style("secondary"))
        row_1.addWidget(new_btn)

        save_btn = QPushButton("Сохранить")
        save_btn.clicked.connect(self.save_program)
        save_btn.setStyleSheet(self.button_style("primary"))
        row_1.addWidget(save_btn)

        layout.addLayout(row_1)

        row_2 = QHBoxLayout()

        load_btn = QPushButton("Открыть")
        load_btn.clicked.connect(self.load_selected_program)
        load_btn.setStyleSheet(self.button_style("secondary"))
        row_2.addWidget(load_btn)

        delete_btn = QPushButton("Удалить")
        delete_btn.clicked.connect(self.delete_selected_program)
        delete_btn.setStyleSheet(self.button_style("danger"))
        row_2.addWidget(delete_btn)

        layout.addLayout(row_2)

        return group

    def build_program_list_panel(self) -> QGroupBox:
        group = QGroupBox("СОХРАНЁННЫЕ ПРОГРАММЫ")
        layout = QVBoxLayout(group)

        self.program_list = QListWidget()
        self.program_list.setStyleSheet(self.list_style())
        self.program_list.itemDoubleClicked.connect(
            lambda _: self.load_selected_program()
        )
        layout.addWidget(self.program_list)

        return group

    def build_help_panel(self) -> QGroupBox:
        group = QGroupBox("КОМАНДЫ")
        layout = QVBoxLayout(group)
        layout.setSpacing(8)

        self.command_combo = QComboBox()
        self.command_combo.setStyleSheet(self.combo_style())
        self.command_combo.addItem("move_lin(dx, dy, dz, duration)")
        self.command_combo.addItem("rotate_rx(angle_deg, duration)")
        self.command_combo.addItem("rotate_ry(angle_deg, duration)")
        self.command_combo.addItem("rotate_rz(angle_deg, duration)")
        self.command_combo.addItem("joint_set(index, angle_deg, duration)")
        self.command_combo.addItem("wait(duration)")
        self.command_combo.addItem("reset_home()")
        self.command_combo.addItem("grip_open()")
        self.command_combo.addItem("grip_close()")
        self.command_combo.addItem("grip_set(opening_m, duration)")
        self.command_combo.addItem("save_ref()")
        self.command_combo.addItem("align_to_ref(duration)")
        self.command_combo.addItem("stop_motion()")
        layout.addWidget(self.command_combo)

        insert_btn = QPushButton("Вставить команду")
        insert_btn.clicked.connect(self.insert_selected_command)
        insert_btn.setStyleSheet(self.button_style("primary"))
        layout.addWidget(insert_btn)

        self.help_display = QPlainTextEdit()
        self.help_display.setReadOnly(True)
        self.help_display.setStyleSheet(self.text_box_style(monospace=True))
        self.help_display.setPlainText(
            "Примеры:\n\n"
            "move_lin(0.01, 0, 0, 1.0)\n"
            "rotate_rz(15, 1.0)\n"
            "joint_set(2, 30, 1.5)\n"
            "wait(1.0)\n"
            "grip_open()\n"
            "grip_close()\n\n"
            "Цикл:\n\n"
            "for i in range(3):\n"
            "    move_lin(0.01, 0, 0, 0.5)"
        )
        layout.addWidget(self.help_display, 1)

        return group

    def build_editor_panel(self) -> QGroupBox:
        group = QGroupBox("КОД ПРОГРАММЫ")
        layout = QVBoxLayout(group)
        layout.setSpacing(8)

        self.editor = QPlainTextEdit()
        self.editor.setStyleSheet(self.code_box_style())
        self.editor.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.editor.textChanged.connect(self.on_editor_changed)
        layout.addWidget(self.editor, 1)

        row = QHBoxLayout()

        validate_btn = QPushButton("Проверить")
        validate_btn.clicked.connect(self.validate_program)
        validate_btn.setStyleSheet(self.button_style("primary"))
        row.addWidget(validate_btn)

        self.validation_status = QLabel("Статус: не проверено")
        self.validation_status.setStyleSheet(
            f"color: {self.COLOR_MUTED}; font-weight: bold;"
        )
        row.addWidget(self.validation_status, 1)

        layout.addLayout(row)

        self.validation_display = QPlainTextEdit()
        self.validation_display.setReadOnly(True)
        self.validation_display.setMaximumHeight(80)
        self.validation_display.setStyleSheet(self.text_box_style(monospace=True))
        layout.addWidget(self.validation_display)

        return group

    def build_execution_panel(self) -> QGroupBox:
        group = QGroupBox("ВЫПОЛНЕНИЕ")
        layout = QVBoxLayout(group)
        layout.setSpacing(8)

        row = QHBoxLayout()

        self.run_btn = QPushButton("▶ Запустить")
        self.run_btn.clicked.connect(self.run_program)
        self.run_btn.setStyleSheet(self.button_style("primary"))
        row.addWidget(self.run_btn)

        self.stop_btn = QPushButton("⏹ Остановить")
        self.stop_btn.clicked.connect(self.stop_program)
        self.stop_btn.setEnabled(False)
        self.stop_btn.setStyleSheet(self.button_style("secondary"))
        row.addWidget(self.stop_btn)

        clear_log_btn = QPushButton("Очистить лог")
        clear_log_btn.clicked.connect(self.clear_log)
        clear_log_btn.setStyleSheet(self.button_style("secondary"))
        row.addWidget(clear_log_btn)

        self.execution_status = QLabel("Статус: ожидание")
        self.execution_status.setStyleSheet(
            f"color: {self.COLOR_MUTED}; font-weight: bold;"
        )
        row.addWidget(self.execution_status, 1)

        layout.addLayout(row)

        self.log_display = QPlainTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setMaximumBlockCount(500)
        self.log_display.setStyleSheet(self.log_box_style())
        layout.addWidget(self.log_display, 1)

        return group

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

    def line_edit_style(self) -> str:
        return f"""
        QLineEdit {{
            background-color: {self.COLOR_PANEL_2};
            color: {self.COLOR_TEXT};
            border: 1px solid {self.COLOR_BORDER};
            border-radius: 8px;
            padding: 8px;
        }}

        QLineEdit:focus {{
            border: 1px solid {self.COLOR_ACCENT};
        }}
        """

    def list_style(self) -> str:
        return f"""
        QListWidget {{
            background-color: {self.COLOR_PANEL_2};
            color: {self.COLOR_TEXT};
            border: 1px solid {self.COLOR_BORDER};
            border-radius: 8px;
            padding: 6px;
        }}

        QListWidget::item {{
            padding: 6px;
            border-radius: 5px;
        }}

        QListWidget::item:selected {{
            background-color: {self.COLOR_ACCENT};
            color: white;
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

    def text_box_style(self, monospace: bool = False) -> str:
        font = "Courier" if monospace else "Arial"
        return f"""
        QPlainTextEdit {{
            background-color: {self.COLOR_PANEL_2};
            color: {self.COLOR_TEXT};
            border: 1px solid {self.COLOR_BORDER};
            border-radius: 8px;
            padding: 8px;
            font-family: {font};
            font-size: 10px;
        }}
        """

    def code_box_style(self) -> str:
        return f"""
        QPlainTextEdit {{
            background-color: {self.COLOR_PANEL_2};
            color: #d7ffd7;
            border: 1px solid {self.COLOR_BORDER};
            border-radius: 8px;
            padding: 10px;
            font-family: 'Courier New', monospace;
            font-size: 12px;
        }}
        """

    def log_box_style(self) -> str:
        return f"""
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

    def refresh_program_list(self) -> None:
        selected_name = self.current_program_name

        self.program_list.clear()

        for name in self.storage.list_programs():
            item = QListWidgetItem(name)
            self.program_list.addItem(item)

            if selected_name == name:
                self.program_list.setCurrentItem(item)

    def get_selected_program_name(self) -> Optional[str]:
        item = self.program_list.currentItem()

        if item is None:
            return None

        return item.text()

    def new_program(self) -> None:
        self.current_program_name = None
        self.program_name_input.clear()
        self.editor.setPlainText(
            "reset_home()\n"
            "wait(1.0)\n"
            "move_lin(0.01, 0.0, 0.0, 1.0)\n"
            "wait(1.0)\n"
            "move_lin(-0.01, 0.0, 0.0, 1.0)\n"
        )
        self.validation_display.clear()
        self.validation_status.setText("Статус: не проверено")
        self.execution_status.setText("Статус: ожидание")
        self.log("Создана новая программа.")

    def load_selected_program(self) -> None:
        name = self.get_selected_program_name()

        if not name:
            QMessageBox.warning(self, "Ошибка", "Выберите программу.")
            return

        self.load_program(name)

    def load_program(self, program_name: str) -> None:
        try:
            source = self.storage.load_program(program_name)
            self.current_program_name = program_name
            self.program_name_input.setText(program_name)
            self.editor.setPlainText(source)
            self.validation_display.clear()
            self.validation_status.setText("Статус: не проверено")
            self.log(f"Программа открыта: {program_name}")
            self.refresh_program_list()

        except Exception as exc:
            QMessageBox.critical(
                self,
                "Ошибка открытия",
                f"Не удалось открыть программу:\n{exc}",
            )

    def save_program(self) -> None:
        name = self.program_name_input.text().strip()

        if not name:
            QMessageBox.warning(self, "Ошибка", "Введите имя программы.")
            return

        source = self.editor.toPlainText()

        try:
            commands = self.parser.parse(source)
            self.storage.save_program(name, source, commands)

            self.current_program_name = name
            self.last_valid_commands = commands

            self.refresh_program_list()
            self.program_saved.emit(name)

            self.validation_status.setText("Статус: OK")
            self.validation_status.setStyleSheet(
                f"color: {self.COLOR_SUCCESS}; font-weight: bold;"
            )
            self.validation_display.setPlainText(
                f"OK: программа сохранена. Команд: {len(commands)}"
            )

            self.log(f"Программа сохранена: {name}")

        except ProgramParseError as exc:
            self.validation_status.setText("Статус: ошибка")
            self.validation_status.setStyleSheet(
                f"color: {self.COLOR_DANGER}; font-weight: bold;"
            )
            self.validation_display.setPlainText(str(exc))
            QMessageBox.critical(self, "Ошибка программы", str(exc))

        except Exception as exc:
            QMessageBox.critical(
                self,
                "Ошибка сохранения",
                f"Не удалось сохранить программу:\n{exc}",
            )

    def delete_selected_program(self) -> None:
        name = self.get_selected_program_name()

        if not name:
            QMessageBox.warning(self, "Ошибка", "Выберите программу.")
            return

        reply = QMessageBox.question(
            self,
            "Удаление программы",
            f"Удалить программу '{name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )

        if reply != QMessageBox.StandardButton.Yes:
            return

        try:
            self.storage.delete_program(name)
            self.program_deleted.emit(name)

            if self.current_program_name == name:
                self.new_program()

            self.refresh_program_list()
            self.log(f"Программа удалена: {name}", "WARN")

        except Exception as exc:
            QMessageBox.critical(
                self,
                "Ошибка удаления",
                f"Не удалось удалить программу:\n{exc}",
            )

    def insert_selected_command(self) -> None:
        command = self.command_combo.currentText()

        if not command:
            return

        cursor = self.editor.textCursor()
        cursor.insertText(command + "\n")
        self.editor.setTextCursor(cursor)
        self.editor.setFocus()

    def on_editor_changed(self) -> None:
        self.validation_status.setText("Статус: изменено")
        self.validation_status.setStyleSheet(
            f"color: {self.COLOR_MUTED}; font-weight: bold;"
        )

    def validate_program(self) -> bool:
        source = self.editor.toPlainText()

        ok, messages, commands = self.parser.validate_source(source)

        self.validation_display.setPlainText("\n".join(messages))

        if ok:
            self.last_valid_commands = commands
            self.validation_status.setText("Статус: OK")
            self.validation_status.setStyleSheet(
                f"color: {self.COLOR_SUCCESS}; font-weight: bold;"
            )
            self.log(f"Проверка успешна. Команд: {len(commands)}")
            return True

        self.last_valid_commands = []
        self.validation_status.setText("Статус: ошибка")
        self.validation_status.setStyleSheet(
            f"color: {self.COLOR_DANGER}; font-weight: bold;"
        )
        self.log("Проверка программы не пройдена.", "ERROR")
        return False

    def run_program(self) -> None:
        if self.is_running:
            QMessageBox.warning(
                self,
                "Выполнение активно",
                "Сначала остановите текущую программу.",
            )
            return

        source = self.editor.toPlainText()

        try:
            commands = self.parser.parse(source)
        except ProgramParseError as exc:
            self.validation_display.setPlainText(str(exc))
            self.validation_status.setText("Статус: ошибка")
            self.validation_status.setStyleSheet(
                f"color: {self.COLOR_DANGER}; font-weight: bold;"
            )
            QMessageBox.critical(self, "Ошибка программы", str(exc))
            self.log(f"Ошибка запуска: {exc}", "ERROR")
            return

        if not commands:
            QMessageBox.warning(
                self,
                "Нет команд",
                "Программа не содержит команд.",
            )
            return

        self.start_worker(commands)

    def start_worker(self, commands: List[ProgramCommand]) -> None:
        self.is_running = True
        self.stop_requested = False

        self.run_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

        self.set_execution_status("выполнение", self.COLOR_ACCENT)
        self.log("Запуск программы.")

        self.executor = RobotProgramExecutor(
            robot=self.robot_bridge,
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

    def stop_program(self) -> None:
        if not self.is_running:
            return

        self.stop_requested = True
        self.stop_btn.setEnabled(False)

        self.log("Остановка программы пользователем.", "WARN")
        self.set_execution_status("остановка...", self.COLOR_WARNING)

        if self.executor is not None:
            self.executor.request_stop()

        try:
            if self.robot_bridge is not None and hasattr(self.robot_bridge, "stop_motion"):
                self.robot_bridge.stop_motion()
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
        self.set_execution_status(f"команда {index}/{total}: {raw}", self.COLOR_ACCENT)

    def on_worker_finished(self, status: str, message: str) -> None:
        self.is_running = False

        self.run_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

        if status == "success":
            self.set_execution_status("завершено", self.COLOR_SUCCESS)
            self.log(message)

        elif status == "stopped":
            self.set_execution_status("остановлено", self.COLOR_WARNING)
            self.log("Программа остановлена пользователем.", "WARN")

        else:
            if self.stop_requested:
                self.set_execution_status("остановлено", self.COLOR_WARNING)
                self.log("Программа остановлена пользователем.", "WARN")
            else:
                self.set_execution_status("ошибка", self.COLOR_DANGER)
                self.log(message, "ERROR")

        self.stop_requested = False
        self.executor = None
        self.worker = None
        self.worker_thread = None

    def set_execution_status(self, text: str, color: str) -> None:
        self.execution_status.setText(f"Статус: {text}")
        self.execution_status.setStyleSheet(
            f"color: {color}; font-weight: bold;"
        )

    def log(self, message: str, level: str = "INFO") -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_display.appendPlainText(f"[{timestamp}] [{level}] {message}")

        scrollbar = self.log_display.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def clear_log(self) -> None:
        self.log_display.clear()

    def closeEvent(self, event) -> None:
        if self.is_running:
            self.stop_program()

        event.accept()