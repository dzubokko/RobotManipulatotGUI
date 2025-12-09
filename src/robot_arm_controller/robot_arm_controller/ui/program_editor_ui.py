import time
import math
import re
import os
import json
from pathlib import Path
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QLineEdit, QPushButton, QTextEdit, QMessageBox, QListWidget,
    QListWidgetItem, QDialog, QComboBox
)
from PyQt6.QtCore import QTimer, Qt
from typing import Dict, List


class ProgramEditorUI(QWidget):
    def __init__(self, robot_controller):
        super().__init__()
        self.robot = robot_controller
        self.execution_context = {}
        
        # ✅ ПАПКА ДЛЯ ПРОГРАММ
        self.programs_dir = Path("programs")
        self.programs_dir.mkdir(exist_ok=True)
        
        self.init_ui()
        
        # ✅ ИСПРАВЛЕНИЕ: размеры окна как в manual_control_ui
        self.setMinimumSize(900, 700)
        self.setGeometry(100, 100, 1200, 900)

    def init_ui(self):
        main_layout = QVBoxLayout()

        # Заголовок
        title = QLabel("💻 РЕДАКТОР ПРОГРАММЫ v4.1 (ФУНКЦИИ + ЦИКЛЫ + СОХРАНЕНИЕ)")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #0078d4;")
        main_layout.addWidget(title)

        # === ПАНЕЛЬ УПРАВЛЕНИЯ ===
        control_panel = QGroupBox("УПРАВЛЕНИЕ ПРОГРАММОЙ")
        control_layout = QHBoxLayout()

        name_label = QLabel("Имя программы:")
        name_label.setStyleSheet("font-weight: bold;")
        control_layout.addWidget(name_label)

        self.program_name_input = QLineEdit()
        self.program_name_input.setPlaceholderText("Введите имя программы...")
        self.program_name_input.setMaximumWidth(300)
        control_layout.addWidget(self.program_name_input)

        new_btn = QPushButton("📄 Новая")
        new_btn.clicked.connect(self.on_new_program)
        control_layout.addWidget(new_btn)

        # ✅ НОВАЯ КНОПКА: список программ
        open_btn = QPushButton("📂 Открыть")
        open_btn.clicked.connect(self.on_open_program_dialog)
        control_layout.addWidget(open_btn)

        save_btn = QPushButton("💾 Сохранить")
        save_btn.clicked.connect(self.on_save_program)
        save_btn.setStyleSheet("background-color: #28a745;")
        control_layout.addWidget(save_btn)

        delete_btn = QPushButton("🗑️ Удалить")
        delete_btn.clicked.connect(self.on_delete_program)
        delete_btn.setStyleSheet("background-color: #dc3545;")
        control_layout.addWidget(delete_btn)

        control_layout.addStretch()
        control_panel.setLayout(control_layout)
        main_layout.addWidget(control_panel)

        # === РЕДАКТОР КОДА ===
        editor_box = QGroupBox("КОД ПРОГРАММЫ")
        editor_layout = QVBoxLayout()

        help_label = QLabel(
            "📝 СИНТАКСИС ПРОГРАММЫ (функции + циклы):\n\n"
            "🔹 ДВИЖЕНИЕ:\n"
            "   move_lin(dx, dy, dz, time) – линейное движение (мировые оси)\n"
            "   rotate_rx(angle_rad, time) – вращение вокруг X без дрейфа\n"
            "   rotate_ry(angle_rad, time) – вращение вокруг Y без дрейфа\n"
            "   rotate_rz(angle_rad, time) – вращение вокруг Z локальная ось\n"
            "   rotate_rp(rx_rad, ry_rad, time) – вращение RX и RY одновременно\n\n"
            "🔹 СУСТАВЫ:\n"
            "   joint_set(idx, angle_deg, time) – установить сустав в позицию\n"
            "   reset_home() – вернуться в исходную позицию\n\n"
            "🔹 ДРУГОЕ:\n"
            "   wait(time_sec) – ожидание (блокирует выполнение!)\n"
            "   grip_open() / grip_close() – захват\n"
            "   save_ref() – сохранить ориентацию\n"
            "   align_to_ref() – подравнять к сохранённой ориентации\n\n"
            "🔹 ЦИКЛЫ: for count in range(N): ... | 🔹 ПЕРЕМЕННЫЕ: $var_name = value"
        )
        help_label.setStyleSheet("color: #666; font-size: 9px; background-color: #f9f9f9; padding: 6px;")
        editor_layout.addWidget(help_label)

        self.code_editor = QTextEdit()
        self.code_editor.setPlaceholderText(
            "# Пример программы v4.1 с функциями и циклом:\n"
            "move_lin(0.1, 0, 0, 2)\n"
            "wait(1)\n"
            "rotate_rx(0.5, 1.5)\n"
            "for count in range(3):\n"
            "    move_lin(0.05, 0, 0, 1)\n"
            "    rotate_rz(0.3, 0.5)\n"
            "    wait(0.5)\n"
            "rotate_ry(0.3, 1)\n"
            "reset_home()"
        )
        self.code_editor.setStyleSheet(
            "background-color: #2d2d2d; color: #00ff00; "
            "font-family: 'Courier New', monospace; font-size: 11px; padding: 10px;"
        )
        self.code_editor.setMinimumHeight(300)
        editor_layout.addWidget(self.code_editor)
        editor_box.setLayout(editor_layout)
        main_layout.addWidget(editor_box)

        # === ПАНЕЛЬ ВЫПОЛНЕНИЯ ===
        execution_box = QGroupBox("ВЫПОЛНЕНИЕ")
        execution_layout = QHBoxLayout()

        run_btn = QPushButton("▶️ ВЫПОЛНИТЬ")
        run_btn.clicked.connect(self.on_run_program)
        run_btn.setStyleSheet(
            "background-color: #28a745; font-weight: bold; font-size: 12px; "
            "border-radius: 5px;"
        )
        run_btn.setMinimumHeight(40)
        execution_layout.addWidget(run_btn)

        stop_btn = QPushButton("⏹️ ОСТАНОВИТЬ")
        stop_btn.clicked.connect(self.on_stop_program)
        stop_btn.setStyleSheet(
            "background-color: #dc3545; font-weight: bold; font-size: 12px; "
            "border-radius: 5px;"
        )
        stop_btn.setMinimumHeight(40)
        execution_layout.addWidget(stop_btn)

        execution_box.setLayout(execution_layout)
        main_layout.addWidget(execution_box)

        # === ЛОГ ВЫПОЛНЕНИЯ ===
        main_layout.addWidget(QLabel("📋 ЛОГ ВЫПОЛНЕНИЯ:"))
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setMaximumHeight(150)
        self.log_display.setStyleSheet(
            "background-color: #1e1e1e; color: #00ff00; "
            "font-family: 'Courier New', monospace; font-size: 10px;"
        )
        main_layout.addWidget(self.log_display)

        # === ТЕРМИНАЛ УПРАВЛЕНИЯ ===
        term_group = QGroupBox("ТЕРМИНАЛ УПРАВЛЕНИЯ")
        term_layout = QHBoxLayout()

        self.term_input = QLineEdit()
        self.term_input.setPlaceholderText(
            "move_lin(0.1, 0, 0, 2) | rotate_rx(0.5, 1) | wait(1)"
        )
        self.term_input.returnPressed.connect(self.on_term_execute)
        term_layout.addWidget(self.term_input)

        term_exec_btn = QPushButton("▶ Выполнить")
        term_exec_btn.clicked.connect(self.on_term_execute)
        term_exec_btn.setStyleSheet("background-color: #0078d4; color: white;")
        term_layout.addWidget(term_exec_btn)

        term_group.setLayout(term_layout)
        main_layout.addWidget(term_group)

        main_layout.addStretch()
        self.setLayout(main_layout)

    # ================== СЛУЖЕБНЫЕ МЕТОДЫ ==================
    def log(self, message: str):
        """Вывести строку в лог выполнения и пролистать вниз."""
        timestamp = time.strftime("%H:%M:%S")
        self.log_display.append(f"[{timestamp}] {message}")
        bar = self.log_display.verticalScrollBar()
        bar.setValue(bar.maximum())

    # ================== РАБОТА С ПРОГРАММАМИ ==================
    def on_new_program(self):
        """Создать новую программу"""
        program_name = self.program_name_input.text().strip()
        if not program_name:
            QMessageBox.warning(self, "Ошибка", "Введите имя программы!")
            return
        try:
            self.code_editor.clear()
            self.log(f"✅ Новая программа: {program_name}")
        except Exception as e:
            QMessageBox.critical(self, "Ошибка", f"Не удалось создать: {e}")

    def on_open_program_dialog(self):
        """✅ Открыть диалог выбора программы"""
        programs = self._get_programs_list()
        
        if not programs:
            QMessageBox.information(self, "Информация", "Нет сохранённых программ!")
            return
        
        # ✅ Создаём диалог с выбором программы
        dialog = QDialog(self)
        dialog.setWindowTitle("📂 Открыть программу")
        dialog.setGeometry(200, 200, 400, 300)
        
        layout = QVBoxLayout()
        
        label = QLabel("Выберите программу для открытия:")
        layout.addWidget(label)
        
        # ✅ Список программ
        program_list = QListWidget()
        for prog in sorted(programs):
            item = QListWidgetItem(prog)
            program_list.addItem(item)
        layout.addWidget(program_list)
        
        # ✅ Кнопки
        btn_layout = QHBoxLayout()
        
        def open_selected():
            if program_list.currentItem():
                selected = program_list.currentItem().text()
                self._load_program(selected)
                dialog.accept()
        
        open_btn = QPushButton("Открыть")
        open_btn.clicked.connect(open_selected)
        btn_layout.addWidget(open_btn)
        
        cancel_btn = QPushButton("Отмена")
        cancel_btn.clicked.connect(dialog.reject)
        btn_layout.addWidget(cancel_btn)
        
        layout.addLayout(btn_layout)
        dialog.setLayout(layout)
        dialog.exec()

    def _get_programs_list(self) -> List[str]:
        """✅ Получить список всех программ в папке programs"""
        programs = []
        if self.programs_dir.exists():
            for file in self.programs_dir.glob("*.robot"):
                programs.append(file.stem)
        return programs

    def _load_program(self, program_name: str):
        """✅ Загрузить программу из файла"""
        try:
            file_path = self.programs_dir / f"{program_name}.robot"
            if not file_path.exists():
                raise FileNotFoundError(f"Файл {file_path} не найден")
            
            with open(file_path, 'r', encoding='utf-8') as f:
                code = f.read()
            
            self.code_editor.setText(code)
            self.program_name_input.setText(program_name)
            self.log(f"✅ Программа загружена: {program_name}")
            QMessageBox.information(self, "Успех", f"Загружена программа: {program_name}")
        except Exception as e:
            QMessageBox.critical(self, "Ошибка", f"Ошибка загрузки: {e}")
            self.log(f"❌ Ошибка загрузки: {e}")

    def on_save_program(self):
        """✅ Сохранить программу в папку programs"""
        program_name = self.program_name_input.text().strip()
        code = self.code_editor.toPlainText()

        if not program_name:
            QMessageBox.warning(self, "Ошибка", "Введите имя программы!")
            return
        if not code.strip():
            QMessageBox.warning(self, "Ошибка", "Программа пуста!")
            return

        try:
            # ✅ Проверяем валидность имени файла
            if any(c in program_name for c in r'\/:*?"<>|'):
                raise ValueError("Имя содержит недопустимые символы")
            
            file_path = self.programs_dir / f"{program_name}.robot"
            
            # ✅ Сохраняем в JSON с метаданными
            program_data = {
                "name": program_name,
                "code": code,
                "created": time.strftime("%Y-%m-%d %H:%M:%S"),
                "version": "4.1"
            }
            
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(code)  # ✅ Сохраняем просто код
            
            self.log(f"✅ Программа сохранена: {program_name}")
            self.log(f"   📁 Путь: {file_path}")
            QMessageBox.information(self, "Успех", f"Сохранено: {program_name}")
        except Exception as e:
            QMessageBox.critical(self, "Ошибка", f"Ошибка сохранения: {e}")
            self.log(f"❌ Ошибка сохранения: {e}")

    def on_delete_program(self):
        """✅ Удалить программу"""
        program_name = self.program_name_input.text().strip()
        
        if not program_name:
            QMessageBox.warning(self, "Ошибка", "Введите имя программы!")
            return
        
        file_path = self.programs_dir / f"{program_name}.robot"
        
        if not file_path.exists():
            QMessageBox.warning(self, "Ошибка", f"Программа '{program_name}' не найдена!")
            return
        
        reply = QMessageBox.question(
            self, "Удалить программу?",
            f"Вы уверены, что хотите удалить '{program_name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            try:
                file_path.unlink()
                self.log(f"✅ Программа удалена: {program_name}")
                self.code_editor.clear()
                self.program_name_input.clear()
                QMessageBox.information(self, "Успех", f"Программа удалена!")
            except Exception as e:
                QMessageBox.critical(self, "Ошибка", f"Ошибка удаления: {e}")
                self.log(f"❌ Ошибка удаления: {e}")

    # ================== ВЫПОЛНЕНИЕ ПРОГРАММЫ ==================
    def on_run_program(self):
        code = self.code_editor.toPlainText()
        if not code.strip():
            QMessageBox.warning(self, "Ошибка", "Программа пуста!")
            return

        self.log_display.clear()
        self.log("🚀 ВЫПОЛНЕНИЕ ПРОГРАММЫ")
        self.log("=" * 70)
        
        self.execution_context = {}

        try:
            self.execute_program(code)
            self.log("=" * 70)
            self.log("✅ Программа завершена успешно!")
        except Exception as e:
            self.log("=" * 70)
            self.log(f"❌ ОШИБКА: {e}")

    def on_stop_program(self):
        self.log("⏹️ Выполнение остановлено пользователем")

    # ================== ПАРСЕР И ИСПОЛНИТЕЛЬ ПРОГРАММЫ ==================
    def execute_program(self, code: str):
        """Парсит и выполняет программу с поддержкой функций и циклов."""
        lines = []
        for line in code.splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            lines.append(line)

        if not lines:
            self.log("⚠️ Программа не содержит команд")
            return

        self.log(f"📊 Найдено строк: {len(lines)}")
        self._execute_block(lines)

    def _execute_block(self, lines: List[str]) -> None:
        """Рекурсивно выполняет блок строк."""
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            
            if line.startswith("for "):
                self.log(f"🔁 Обнаружен цикл: {line}")
                i, body_lines = self._parse_for_loop(lines, i)
                self._execute_for_loop(line, body_lines)
                continue
            
            if "(" in line and ")" in line:
                self.log(f"▶ {line}")
                try:
                    self._execute_function_call(line)
                    self.log(f"   ✅ Выполнено")
                except Exception as e:
                    self.log(f"   ❌ Ошибка: {e}")
                    raise
            
            i += 1

    def _parse_for_loop(self, lines: List[str], loop_start_idx: int):
        """Парсит цикл for и возвращает индекс после цикла и тело цикла."""
        loop_line = lines[loop_start_idx].strip()
        
        match = re.match(r"for\s+(\w+)\s+in\s+range\s*\(\s*(\d+)\s*\)", loop_line)
        if not match:
            raise ValueError(f"Неверный синтаксис цикла: {loop_line}")
        
        body_lines = []
        i = loop_start_idx + 1
        while i < len(lines):
            line = lines[i]
            if line and not line[0].isspace():
                break
            if line.strip():
                body_lines.append(line.strip())
            i += 1
        
        return i, body_lines

    def _execute_for_loop(self, loop_line: str, body_lines: List[str]):
        """Выполняет цикл for...range()."""
        match = re.match(r"for\s+(\w+)\s+in\s+range\s*\(\s*(\d+)\s*\)", loop_line)
        var_name, count_str = match.groups()
        count = int(count_str)
        
        for iteration in range(count):
            self.log(f"   🔄 Итерация {iteration + 1}/{count}")
            self.execution_context[var_name] = iteration
            try:
                self._execute_block(body_lines)
            except Exception as e:
                self.log(f"   ❌ Ошибка в цикле: {e}")
                raise

    def _execute_function_call(self, func_str: str):
        """Парсит и выполняет одну функцию."""
        func_str = func_str.strip()
        
        match = re.match(r"(\w+)\s*\((.*)\)", func_str)
        if not match:
            raise ValueError(f"Неверный синтаксис функции: {func_str}")
        
        func_name, args_str = match.groups()
        args = self._parse_arguments(args_str)
        
        if func_name == "move_lin":
            if len(args) != 4:
                raise ValueError(f"move_lin требует 4 аргумента, получено {len(args)}")
            dx, dy, dz, time_duration = args
            self.robot.move_end_effector_world(dx, dy, dz, time_duration)
            time.sleep(time_duration + 0.1)
        
        elif func_name == "rotate_rx":
            if len(args) != 2:
                raise ValueError(f"rotate_rx требует 2 аргумента, получено {len(args)}")
            angle, time_duration = args
            self.robot.rotate_end_effector_rx_ry_ik(d_rx=angle, d_ry=0.0, duration=time_duration)
            time.sleep(time_duration + 0.1)
        
        elif func_name == "rotate_ry":
            if len(args) != 2:
                raise ValueError(f"rotate_ry требует 2 аргумента, получено {len(args)}")
            angle, time_duration = args
            self.robot.rotate_end_effector_rx_ry_ik(d_rx=0.0, d_ry=angle, duration=time_duration)
            time.sleep(time_duration + 0.1)
        
        elif func_name == "rotate_rz":
            if len(args) != 2:
                raise ValueError(f"rotate_rz требует 2 аргумента, получено {len(args)}")
            angle, time_duration = args
            self.robot.rotate_end_effector_world(angle, time_duration)
            time.sleep(time_duration + 0.1)
        
        elif func_name == "rotate_rp":
            if len(args) != 3:
                raise ValueError(f"rotate_rp требует 3 аргумента, получено {len(args)}")
            rx, ry, time_duration = args
            self.robot.rotate_end_effector_rx_ry_ik(d_rx=rx, d_ry=ry, duration=time_duration)
            time.sleep(time_duration + 0.1)
        
        elif func_name == "joint_set":
            if len(args) != 3:
                raise ValueError(f"joint_set требует 3 аргумента, получено {len(args)}")
            idx, angle_deg, time_duration = args
            angle_rad = math.radians(angle_deg)
            self.robot.move_joint(int(idx), angle_rad, time_duration)
            time.sleep(time_duration + 0.1)
        
        elif func_name == "reset_home":
            self.robot.reset_position()
            self.log(f"   🏠 Возврат в исходную позицию")
            time.sleep(2.1)
        
        elif func_name == "wait":
            if len(args) != 1:
                raise ValueError(f"wait требует 1 аргумент, получено {len(args)}")
            time_val = args[0]
            self.log(f"   ⏳ Ожидание {time_val} сек...")
            import time as time_module
            start = time_module.time()
            while time_module.time() - start < time_val:
                from PyQt6.QtCore import QCoreApplication
                QCoreApplication.processEvents()
                time_module.sleep(0.01)
        
        elif func_name == "grip_open":
            self.log(f"   🖐️ Захват открыт")
            time.sleep(0.5)
        
        elif func_name == "grip_close":
            self.log(f"   ✊ Захват закрыт")
            time.sleep(0.5)
        
        elif func_name == "save_ref":
            self.robot.save_reference_orientation()
            self.log(f"   💾 Ориентация сохранена")
            time.sleep(0.2)
        
        elif func_name == "align_to_ref":
            duration = args[0] if len(args) > 0 else 0.3
            self.robot.align_orientation_to_reference(duration)
            self.log(f"   🎯 Подравнивание к ориентации")
            time.sleep(duration + 0.1)
        
        else:
            raise ValueError(f"❓ Неизвестная функция: {func_name}")

    def _parse_arguments(self, args_str: str) -> List[float]:
        """Парсит строку аргументов и возвращает список чисел."""
        if not args_str.strip():
            return []
        
        parts = args_str.split(",")
        result = []
        
        for part in parts:
            part = part.strip()
            try:
                val = float(part)
                result.append(val)
            except ValueError:
                if part.startswith("$"):
                    var_name = part[1:]
                    if var_name in self.execution_context:
                        result.append(float(self.execution_context[var_name]))
                    else:
                        raise ValueError(f"Переменная {part} не определена")
                else:
                    raise ValueError(f"Неверный аргумент: {part}")
        
        return result

    # ================== ТЕРМИНАЛ ==================
    def on_term_execute(self):
        cmd = self.term_input.text().strip()
        if not cmd:
            return
        
        self.log(f"> {cmd}")
        try:
            commands = cmd.split("|")
            for single_cmd in commands:
                single_cmd = single_cmd.strip()
                if single_cmd:
                    self._execute_function_call(single_cmd)
                    self.log(f"   ✅ Выполнено")
        except Exception as e:
            self.log(f"❌ {e}")
        
        self.term_input.clear()
