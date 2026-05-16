import json
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

try:
    from robot_arm_controller.core.program_executor import RobotProgramParser
except Exception:
    RobotProgramParser = None


class ProgramManagerUI(QWidget):
    program_open_requested = pyqtSignal(str)
    program_deleted = pyqtSignal(str)
    program_exported = pyqtSignal(str)

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
    COLOR_SUCCESS = "#2da44e"

    def __init__(self, robot_bridge=None):
        super().__init__()

        self.robot_bridge = robot_bridge

        package_root = Path(__file__).resolve().parents[1]
        self.programs_dir = package_root / "programs"
        self.programs_dir.mkdir(parents=True, exist_ok=True)

        self.parser = RobotProgramParser() if RobotProgramParser is not None else None

        self.program_data: Dict[str, Dict] = {}
        self.current_program: Optional[str] = None

        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(5000)
        self.refresh_timer.timeout.connect(self.auto_refresh)

        self.init_ui()
        self.refresh_programs()

        self.refresh_timer.start()

    def init_ui(self) -> None:
        self.setStyleSheet(self.base_stylesheet())

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(12, 10, 12, 10)
        main_layout.setSpacing(10)

        title = QLabel("МЕНЕДЖЕР ПРОГРАММ")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setStyleSheet(f"color: {self.COLOR_TEXT};")
        main_layout.addWidget(title)

        self.main_splitter = QSplitter(Qt.Orientation.Horizontal)

        left_panel = self.build_left_panel()
        right_panel = self.build_right_panel()

        self.main_splitter.addWidget(left_panel)
        self.main_splitter.addWidget(right_panel)
        self.main_splitter.setStretchFactor(0, 2)
        self.main_splitter.setStretchFactor(1, 3)
        self.main_splitter.setSizes([620, 720])

        main_layout.addWidget(self.main_splitter, 1)

        self.setMinimumSize(760, 520)
        self.setLayout(main_layout)

    def build_left_panel(self) -> QWidget:
        panel = QWidget()
        panel.setMinimumWidth(360)

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 6, 0)
        layout.setSpacing(10)

        layout.addWidget(self.build_toolbar())
        layout.addWidget(self.build_table(), 1)
        layout.addWidget(self.build_stats_panel())

        return panel

    def build_right_panel(self) -> QWidget:
        panel = QWidget()
        panel.setMinimumWidth(360)

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(6, 0, 0, 0)
        layout.setSpacing(10)

        layout.addWidget(self.build_info_panel())
        layout.addWidget(self.build_preview_panel(), 1)

        return panel

    def build_toolbar(self) -> QGroupBox:
        group = QGroupBox("УПРАВЛЕНИЕ")
        layout = QVBoxLayout(group)
        layout.setSpacing(8)

        row_1 = QHBoxLayout()

        refresh_btn = QPushButton("Обновить")
        refresh_btn.clicked.connect(self.refresh_programs)
        refresh_btn.setStyleSheet(self.button_style("primary"))
        row_1.addWidget(refresh_btn)

        open_editor_btn = QPushButton("Открыть в редакторе")
        open_editor_btn.clicked.connect(self.on_open_in_editor)
        open_editor_btn.setStyleSheet(self.button_style("primary"))
        row_1.addWidget(open_editor_btn)

        layout.addLayout(row_1)

        row_2 = QHBoxLayout()

        import_btn = QPushButton("Импорт")
        import_btn.clicked.connect(self.on_import_program)
        import_btn.setStyleSheet(self.button_style("secondary"))
        row_2.addWidget(import_btn)

        export_btn = QPushButton("Экспорт")
        export_btn.clicked.connect(self.on_export_program)
        export_btn.setStyleSheet(self.button_style("secondary"))
        row_2.addWidget(export_btn)

        delete_btn = QPushButton("Удалить")
        delete_btn.clicked.connect(self.on_delete_program)
        delete_btn.setStyleSheet(self.button_style("danger"))
        row_2.addWidget(delete_btn)

        folder_btn = QPushButton("Папка")
        folder_btn.clicked.connect(self.on_open_folder)
        folder_btn.setStyleSheet(self.button_style("secondary"))
        row_2.addWidget(folder_btn)

        layout.addLayout(row_2)

        return group

    def build_table(self) -> QGroupBox:
        group = QGroupBox("СПИСОК ПРОГРАММ")
        layout = QVBoxLayout(group)

        self.table = QTableWidget()
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels(
            [
                "Имя",
                "Статус",
                "Команд",
                "Изменена",
                "Размер",
            ]
        )

        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(4, QHeaderView.ResizeMode.ResizeToContents)

        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.itemSelectionChanged.connect(self.on_program_selected)
        self.table.itemDoubleClicked.connect(lambda _: self.on_open_in_editor())
        self.table.setStyleSheet(self.table_style())

        layout.addWidget(self.table)

        return group

    def build_info_panel(self) -> QGroupBox:
        group = QGroupBox("ИНФОРМАЦИЯ")
        layout = QVBoxLayout(group)
        layout.setSpacing(8)

        self.info_display = QPlainTextEdit()
        self.info_display.setReadOnly(True)
        self.info_display.setMaximumHeight(105)
        self.info_display.setStyleSheet(self.text_box_style(monospace=True))
        layout.addWidget(self.info_display)

        desc_row = QHBoxLayout()

        self.description_input = QLineEdit()
        self.description_input.setPlaceholderText("Описание выбранной программы...")
        self.description_input.returnPressed.connect(self.save_description)
        self.description_input.setStyleSheet(self.line_edit_style())
        desc_row.addWidget(self.description_input, 1)

        save_desc_btn = QPushButton("Сохранить")
        save_desc_btn.clicked.connect(self.save_description)
        save_desc_btn.setStyleSheet(self.button_style("primary"))
        desc_row.addWidget(save_desc_btn)

        layout.addLayout(desc_row)

        self.description_status = QLabel("")
        self.description_status.setStyleSheet(
            f"color: {self.COLOR_SUCCESS}; font-weight: bold; font-size: 10px;"
        )
        layout.addWidget(self.description_status)

        return group

    def build_preview_panel(self) -> QGroupBox:
        group = QGroupBox("ПРОСМОТР ФАЙЛА")
        layout = QVBoxLayout(group)
        layout.setSpacing(8)

        self.preview_display = QPlainTextEdit()
        self.preview_display.setReadOnly(True)
        self.preview_display.setStyleSheet(self.code_box_style())
        layout.addWidget(self.preview_display, 1)

        self.validation_display = QPlainTextEdit()
        self.validation_display.setReadOnly(True)
        self.validation_display.setMaximumHeight(75)
        self.validation_display.setStyleSheet(self.text_box_style(monospace=True))
        layout.addWidget(self.validation_display)

        return group

    def build_stats_panel(self) -> QGroupBox:
        group = QGroupBox("СТАТИСТИКА")
        layout = QVBoxLayout(group)

        self.total_label = QLabel("Всего программ: 0")
        self.total_label.setStyleSheet(
            f"color: {self.COLOR_TEXT}; font-weight: bold; font-size: 11px;"
        )
        layout.addWidget(self.total_label)

        self.size_label = QLabel("Общий размер: 0 КБ")
        self.size_label.setStyleSheet(
            f"color: {self.COLOR_MUTED}; font-weight: bold; font-size: 11px;"
        )
        layout.addWidget(self.size_label)

        self.path_label = QLabel(f"Папка: {self.programs_dir}")
        self.path_label.setWordWrap(True)
        self.path_label.setStyleSheet(f"color: {self.COLOR_MUTED}; font-size: 9px;")
        layout.addWidget(self.path_label)

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

    def table_style(self) -> str:
        return f"""
        QTableWidget {{
            background-color: {self.COLOR_PANEL_2};
            color: {self.COLOR_TEXT};
            gridline-color: {self.COLOR_BORDER};
            border: 1px solid {self.COLOR_BORDER};
            border-radius: 8px;
            selection-background-color: {self.COLOR_ACCENT};
        }}

        QHeaderView::section {{
            background-color: {self.COLOR_PANEL};
            color: {self.COLOR_TEXT};
            padding: 6px;
            border: none;
            border-bottom: 1px solid {self.COLOR_BORDER};
            font-weight: bold;
        }}

        QTableWidget::item:selected {{
            background-color: {self.COLOR_ACCENT};
            color: white;
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

    def refresh_programs(self) -> None:
        selected_before = self.current_program

        self.program_data.clear()
        self.table.setRowCount(0)

        self.programs_dir.mkdir(parents=True, exist_ok=True)

        files = sorted(self.programs_dir.glob("*.robot"))
        total_size = 0
        programs: List[Dict] = []

        for path in files:
            info = self.read_program_info(path)
            programs.append(info)
            total_size += info["size"]

        self.table.setRowCount(len(programs))

        for row, info in enumerate(programs):
            self.program_data[info["name"]] = info

            name_item = QTableWidgetItem(info["name"])
            name_item.setFont(QFont("Arial", 10, QFont.Weight.Bold))
            self.table.setItem(row, 0, name_item)

            status_item = QTableWidgetItem(info["status"])
            self.table.setItem(row, 1, status_item)

            count_item = QTableWidgetItem(str(info["command_count"]))
            count_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.table.setItem(row, 2, count_item)

            modified_item = QTableWidgetItem(info["modified_text"])
            self.table.setItem(row, 3, modified_item)

            size_item = QTableWidgetItem(f"{info['size'] / 1024:.1f} КБ")
            size_item.setTextAlignment(
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
            )
            self.table.setItem(row, 4, size_item)

        self.total_label.setText(f"Всего программ: {len(programs)}")
        self.size_label.setText(f"Общий размер: {total_size / 1024:.2f} КБ")

        if selected_before and selected_before in self.program_data:
            self.select_program_by_name(selected_before)
        elif programs:
            self.table.selectRow(0)
        else:
            self.clear_selection_details()

    def read_program_info(self, path: Path) -> Dict:
        name = path.stem
        size = path.stat().st_size
        modified_ts = path.stat().st_mtime
        modified_text = time.strftime("%d.%m.%Y %H:%M", time.localtime(modified_ts))

        description = ""
        source = ""
        format_name = "plain"
        command_count = 0
        status = "OK"
        validation_messages: List[str] = []

        try:
            raw_text = path.read_text(encoding="utf-8")

            try:
                data = json.loads(raw_text)

                if isinstance(data, dict):
                    format_name = "json"
                    description = str(data.get("description", ""))

                    source_candidate = data.get("source")
                    if isinstance(source_candidate, str):
                        source = source_candidate
                    else:
                        commands_candidate = data.get("commands", [])
                        source = self.json_commands_to_source(commands_candidate)
                else:
                    format_name = "json"
                    source = raw_text

            except json.JSONDecodeError:
                format_name = "plain"
                source = raw_text

            if self.parser is not None:
                ok, messages, commands = self.parser.validate_source(source)
                validation_messages = messages
                command_count = len(commands)

                if not ok:
                    status = "ERROR"
            else:
                validation_messages = ["Парсер program_executor.py недоступен."]
                status = "UNKNOWN"

        except Exception as exc:
            status = "ERROR"
            validation_messages = [str(exc)]
            source = ""

        return {
            "name": name,
            "path": path,
            "size": size,
            "modified_ts": modified_ts,
            "modified_text": modified_text,
            "description": description,
            "source": source,
            "format": format_name,
            "command_count": command_count,
            "status": status,
            "validation_messages": validation_messages,
        }

    def auto_refresh(self) -> None:
        actual_count = len(list(self.programs_dir.glob("*.robot")))
        if actual_count != self.table.rowCount():
            self.refresh_programs()

    def select_program_by_name(self, name: str) -> None:
        for row in range(self.table.rowCount()):
            item = self.table.item(row, 0)
            if item is not None and item.text() == name:
                self.table.selectRow(row)
                return

    def on_program_selected(self) -> None:
        row = self.table.currentRow()

        if row < 0:
            self.clear_selection_details()
            return

        item = self.table.item(row, 0)
        if item is None:
            self.clear_selection_details()
            return

        name = item.text()
        self.current_program = name

        info = self.program_data.get(name)
        if info is None:
            self.clear_selection_details()
            return

        self.update_details(info)

    def update_details(self, info: Dict) -> None:
        self.description_input.setText(info["description"])
        self.description_status.clear()

        self.info_display.setPlainText(
            f"Программа: {info['name']}\n"
            f"Формат: {info['format']}\n"
            f"Статус: {info['status']}\n"
            f"Команд: {info['command_count']}\n"
            f"Размер: {info['size']} байт"
        )

        self.preview_display.setPlainText(info["source"])
        self.validation_display.setPlainText("\n".join(info["validation_messages"]))

    def clear_selection_details(self) -> None:
        self.current_program = None
        self.description_input.clear()
        self.description_status.clear()
        self.info_display.clear()
        self.preview_display.clear()
        self.validation_display.clear()

    def get_selected_program_name(self) -> Optional[str]:
        return self.current_program

    def get_selected_program_info(self) -> Optional[Dict]:
        if not self.current_program:
            return None

        return self.program_data.get(self.current_program)

    def save_description(self) -> None:
        info = self.get_selected_program_info()

        if info is None:
            QMessageBox.warning(self, "Ошибка", "Выберите программу.")
            return

        description = self.description_input.text().strip()
        path: Path = info["path"]

        try:
            raw_text = path.read_text(encoding="utf-8")

            try:
                data = json.loads(raw_text)
                if not isinstance(data, dict):
                    data = {}
            except json.JSONDecodeError:
                data = {}

            source = data.get("source")
            if not isinstance(source, str):
                source = info["source"]

            commands_json = []
            if self.parser is not None:
                commands = self.parser.parse(source)
                commands_json = self.commands_to_json(commands)

            data["name"] = info["name"]
            data["format"] = "json"
            data["description"] = description
            data["source"] = source
            data["commands"] = commands_json

            path.write_text(
                json.dumps(data, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )

            self.description_status.setText("Описание сохранено")
            QTimer.singleShot(2000, lambda: self.description_status.clear())

            self.current_program = info["name"]
            self.refresh_programs()

        except Exception as exc:
            QMessageBox.critical(
                self,
                "Ошибка сохранения",
                f"Не удалось сохранить описание:\n{exc}",
            )
            self.description_status.setText("Ошибка сохранения")

    def on_open_in_editor(self) -> None:
        name = self.get_selected_program_name()

        if not name:
            QMessageBox.warning(self, "Ошибка", "Выберите программу.")
            return

        self.program_open_requested.emit(name)

    def on_delete_program(self) -> None:
        info = self.get_selected_program_info()

        if info is None:
            QMessageBox.warning(self, "Ошибка", "Выберите программу для удаления.")
            return

        reply = QMessageBox.question(
            self,
            "Удаление программы",
            f"Удалить программу '{info['name']}'?\n\nЭто действие невозможно отменить.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )

        if reply != QMessageBox.StandardButton.Yes:
            return

        try:
            info["path"].unlink()
            self.program_deleted.emit(info["name"])
            self.current_program = None
            self.refresh_programs()

        except Exception as exc:
            QMessageBox.critical(
                self,
                "Ошибка удаления",
                f"Не удалось удалить программу:\n{exc}",
            )

    def on_export_program(self) -> None:
        info = self.get_selected_program_info()

        if info is None:
            QMessageBox.warning(self, "Ошибка", "Выберите программу для экспорта.")
            return

        export_path, _ = QFileDialog.getSaveFileName(
            self,
            "Экспортировать программу",
            f"{info['name']}.robot",
            "Robot Files (*.robot);;JSON Files (*.json);;All Files (*)",
        )

        if not export_path:
            return

        try:
            shutil.copy2(info["path"], export_path)
            self.program_exported.emit(info["name"])

            QMessageBox.information(
                self,
                "Экспорт",
                f"Программа экспортирована:\n{export_path}",
            )

        except Exception as exc:
            QMessageBox.critical(
                self,
                "Ошибка экспорта",
                f"Не удалось экспортировать программу:\n{exc}",
            )

    def on_import_program(self) -> None:
        import_path, _ = QFileDialog.getOpenFileName(
            self,
            "Импортировать программу",
            "",
            "Robot Files (*.robot);;JSON Files (*.json);;All Files (*)",
        )

        if not import_path:
            return

        source_path = Path(import_path)

        try:
            raw_text = source_path.read_text(encoding="utf-8")
            program_source = raw_text

            try:
                data = json.loads(raw_text)

                if isinstance(data, dict) and isinstance(data.get("source"), str):
                    program_source = data["source"]
                elif isinstance(data, dict) and isinstance(data.get("commands"), list):
                    program_source = self.json_commands_to_source(data["commands"])
            except json.JSONDecodeError:
                pass

            if self.parser is not None:
                self.parser.parse(program_source)

            dest = self.programs_dir / f"{source_path.stem}.robot"

            if dest.exists():
                base = dest.stem
                counter = 1

                while dest.exists():
                    dest = self.programs_dir / f"{base}_{counter}.robot"
                    counter += 1

            shutil.copy2(source_path, dest)

            self.current_program = dest.stem
            self.refresh_programs()

            QMessageBox.information(
                self,
                "Импорт",
                f"Программа импортирована:\n{dest}",
            )

        except Exception as exc:
            QMessageBox.critical(
                self,
                "Ошибка импорта",
                f"Не удалось импортировать программу:\n{exc}",
            )

    def on_open_folder(self) -> None:
        try:
            if sys.platform.startswith("win"):
                import os

                os.startfile(str(self.programs_dir))
            elif sys.platform == "darwin":
                subprocess.run(["open", str(self.programs_dir)], check=False)
            else:
                subprocess.run(["xdg-open", str(self.programs_dir)], check=False)

        except Exception as exc:
            QMessageBox.critical(
                self,
                "Ошибка",
                f"Не удалось открыть папку:\n{exc}",
            )

    def get_current_program(self):
        return self.get_selected_program_info()

    def get_all_programs(self):
        return list(self.program_data.keys())

    def load_program(self, program_name: str):
        return self.program_data.get(program_name)

    def get_programs_dir(self):
        return str(self.programs_dir)

    def closeEvent(self, event):
        self.refresh_timer.stop()
        event.accept()

    @staticmethod
    def commands_to_json(commands) -> List[Dict]:
        result = []

        for command in commands:
            result.append(
                {
                    "line_no": int(command.line_no),
                    "name": str(command.name),
                    "args": [float(arg) for arg in command.args],
                    "raw": str(command.raw),
                }
            )

        return result

    @staticmethod
    def json_commands_to_source(commands: List[Dict]) -> str:
        lines: List[str] = []

        for command in commands:
            if not isinstance(command, dict):
                continue

            raw = command.get("raw")
            if isinstance(raw, str) and raw.strip():
                lines.append(raw.strip())
                continue

            name = command.get("name")
            args = command.get("args", [])

            if isinstance(name, str) and isinstance(args, list):
                args_text = ", ".join(str(arg) for arg in args)
                lines.append(f"{name}({args_text})")
                continue

            ctype = command.get("type")

            if ctype == "move":
                dx, dy, dz = command.get("target", [0.0, 0.0, 0.0])
                duration = command.get("speed", 1.0)
                lines.append(f"move_lin({dx}, {dy}, {dz}, {duration})")

            elif ctype == "rotate_rx":
                lines.append(
                    f"rotate_rx({command.get('angle', 0.0)}, {command.get('duration', 1.0)})"
                )

            elif ctype == "rotate_ry":
                lines.append(
                    f"rotate_ry({command.get('angle', 0.0)}, {command.get('duration', 1.0)})"
                )

            elif ctype == "rotate_rz":
                lines.append(
                    f"rotate_rz({command.get('angle', 0.0)}, {command.get('duration', 1.0)})"
                )

            elif ctype == "joint_set":
                lines.append(
                    f"joint_set({command.get('joint_idx', 1)}, "
                    f"{command.get('angle', 0.0)}, "
                    f"{command.get('duration', 1.0)})"
                )

            elif ctype == "reset_home":
                lines.append("reset_home()")

            elif ctype == "wait":
                lines.append(f"wait({command.get('duration', 1.0)})")

            elif ctype == "gripper_open":
                lines.append("grip_open()")

            elif ctype == "gripper_close":
                lines.append("grip_close()")

            elif ctype == "save_ref":
                lines.append("save_ref()")

            elif ctype == "align_to_ref":
                lines.append(f"align_to_ref({command.get('duration', 0.3)})")

        return "\n".join(lines)

    @staticmethod
    def legacy_json_commands_to_source(commands: List[Dict]) -> str:
        return ProgramManagerUI.json_commands_to_source(commands)