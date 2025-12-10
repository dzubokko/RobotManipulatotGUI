from PyQt6.QtWidgets import *
from PyQt6.QtCore import pyqtSignal, QTimer
from PyQt6.QtGui import QFont
from pathlib import Path
import json
from datetime import datetime
import shutil
import os


class ProgramManagerUI(QWidget):
    """Интерфейс менеджера программ"""
    
    program_selected = pyqtSignal(str)
    program_deleted = pyqtSignal(str)
    program_exported = pyqtSignal(str)
    
    def __init__(self, robot_bridge=None):
        super().__init__()
        self.robot_bridge = robot_bridge
        
        # Путь к папке 
        self.programs_dir = Path(
            "/home/dzubokko/RobotManipulator/src/robot_arm_controller/robot_arm_controller/programs"
        )
        self.programs_dir.mkdir(parents=True, exist_ok=True)
        
        self.current_program = None
        self.program_data = {}
        
        # Автообновление списка
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.auto_refresh)
        self.refresh_timer.start(5000)
        
        self.init_ui()
        self.refresh_programs()
    
    def init_ui(self):
        """Инициализировать UI"""
        main_layout = QVBoxLayout()
        
        # Заголовок
        title = QLabel("МЕНЕДЖЕР ПРОГРАММ")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setStyleSheet("color: #0078d4; margin-bottom: 10px;")
        main_layout.addWidget(title)
        
        # Панель управления
        ctrl_group = QGroupBox("УПРАВЛЕНИЕ")
        ctrl_layout = QHBoxLayout()
        
        refresh_btn = QPushButton("Обновить")
        refresh_btn.setMinimumWidth(100)
        refresh_btn.clicked.connect(self.refresh_programs)
        refresh_btn.setStyleSheet(
            "QPushButton { background-color: #0078d4; color: white; border: none; "
            "padding: 5px; border-radius: 3px; font-weight: bold; }"
            "QPushButton:hover { background-color: #106ebe; }"
            "QPushButton:pressed { background-color: #005a9e; }"
        )
        ctrl_layout.addWidget(refresh_btn)
        
        delete_btn = QPushButton("Удалить")
        delete_btn.setMinimumWidth(100)
        delete_btn.setStyleSheet(
            "QPushButton { background-color: #dc3545; color: white; border: none; "
            "padding: 5px; border-radius: 3px; font-weight: bold; }"
            "QPushButton:hover { background-color: #e74c5c; }"
            "QPushButton:pressed { background-color: #c82333; }"
        )
        delete_btn.clicked.connect(self.on_delete_program)
        ctrl_layout.addWidget(delete_btn)
        
        export_btn = QPushButton("Экспорт")
        export_btn.setMinimumWidth(100)
        export_btn.clicked.connect(self.on_export_program)
        export_btn.setStyleSheet(
            "QPushButton { background-color: #28a745; color: white; border: none; "
            "padding: 5px; border-radius: 3px; font-weight: bold; }"
            "QPushButton:hover { background-color: #34b854; }"
            "QPushButton:pressed { background-color: #1e8449; }"
        )
        ctrl_layout.addWidget(export_btn)
        
        import_btn = QPushButton("Импорт")
        import_btn.setMinimumWidth(100)
        import_btn.clicked.connect(self.on_import_program)
        import_btn.setStyleSheet(
            "QPushButton { background-color: #6c757d; color: white; border: none; "
            "padding: 5px; border-radius: 3px; font-weight: bold; }"
            "QPushButton:hover { background-color: #7f8790; }"
            "QPushButton:pressed { background-color: #545b62; }"
        )
        ctrl_layout.addWidget(import_btn)
        
        folder_btn = QPushButton("Папка")
        folder_btn.setMinimumWidth(100)
        folder_btn.clicked.connect(self.on_open_folder)
        folder_btn.setStyleSheet(
            "QPushButton { background-color: #fd7e14; color: white; border: none; "
            "padding: 5px; border-radius: 3px; font-weight: bold; }"
            "QPushButton:hover { background-color: #fe8c1f; }"
            "QPushButton:pressed { background-color: #e56d04; }"
        )
        ctrl_layout.addWidget(folder_btn)
        
        ctrl_layout.addStretch()
        ctrl_group.setLayout(ctrl_layout)
        main_layout.addWidget(ctrl_group)
        
        # Таблица
        self.table = QTableWidget()
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels([
            "Имя программы", "Создана", "Команд", "Описание", "Размер (байт)"
        ])
        
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(4, QHeaderView.ResizeMode.ResizeToContents)
        
        self.table.setStyleSheet(
            "QTableWidget { "
            "    background-color: #2d2d2d; "
            "    color: #ffffff; "
            "    gridline-color: #444444; "
            "    border: 1px solid #0078d4; "
            "} "
            "QHeaderView::section { "
            "    background-color: #0078d4; "
            "    color: white; "
            "    padding: 5px; "
            "    border: none; "
            "    font-weight: bold; "
            "} "
            "QTableWidget::item:selected { "
            "    background-color: #0d47a1; "
            "} "
            "QTableWidget::item:hover { "
            "    background-color: #1a5490; "
            "}"
        )
        
        self.table.setMinimumHeight(300)
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.table.itemSelectionChanged.connect(self.on_program_selected)
        self.table.setAlternatingRowColors(True)
        
        main_layout.addWidget(self.table)
        
        # Описание
        desc_group = QGroupBox("ОПИСАНИЕ ПРОГРАММЫ")
        desc_layout = QVBoxLayout()
        
        self.description_input = QLineEdit()
        self.description_input.setPlaceholderText("Нажмите на программу в таблице, отредактируйте описание и нажмите Enter...")
        self.description_input.setStyleSheet(
            "QLineEdit { "
            "    background-color: #fff; "
            "    color: #000; "
            "    border: 1px solid #ccc; "
            "    border-radius: 4px; "
            "    padding: 8px; "
            "    font-size: 11px; "
            "}"
        )
        self.description_input.returnPressed.connect(self.save_description)
        desc_layout.addWidget(self.description_input)
        
        # Статус сохранения
        self.save_status = QLabel("")
        self.save_status.setStyleSheet("color: #28a745; font-weight: bold; font-size: 10px;")
        desc_layout.addWidget(self.save_status)
        
        desc_group.setLayout(desc_layout)
        main_layout.addWidget(desc_group)
        
        # Информация
        info_group = QGroupBox("ИНФОРМАЦИЯ")
        info_layout = QVBoxLayout()
        
        self.info_text = QTextEdit()
        self.info_text.setReadOnly(True)
        self.info_text.setMaximumHeight(100)
        self.info_text.setStyleSheet(
            "QTextEdit { "
            "    background-color: #1e1e1e; "
            "    color: #00ff00; "
            "    font-family: 'Courier New', monospace; "
            "    font-size: 9px; "
            "    border: 1px solid #0078d4; "
            "    padding: 5px; "
            "}"
        )
        info_layout.addWidget(self.info_text)
        info_group.setLayout(info_layout)
        main_layout.addWidget(info_group)
        
        # Статистика
        stats_layout = QHBoxLayout()
        
        self.total_label = QLabel("Всего программ: 0")
        self.total_label.setStyleSheet(
            "color: #0078d4; font-weight: bold; font-size: 11px;"
        )
        stats_layout.addWidget(self.total_label)
        
        self.size_label = QLabel("Общий размер: 0 КБ")
        self.size_label.setStyleSheet(
            "color: #28a745; font-weight: bold; font-size: 11px;"
        )
        stats_layout.addWidget(self.size_label)
        
        self.path_label = QLabel(f"📁 Папка: {self.programs_dir}")
        self.path_label.setStyleSheet(
            "color: #666666; font-size: 9px;"
        )
        stats_layout.addWidget(self.path_label)
        
        stats_layout.addStretch()
        main_layout.addLayout(stats_layout)
        
        main_layout.addStretch()
        self.setLayout(main_layout)
    
    def refresh_programs(self):
        """Обновить список программ из папки programs"""
        programs = []
        total_size = 0
        self.program_data = {}
        
        if not self.programs_dir.exists():
            self.programs_dir.mkdir(parents=True, exist_ok=True)
            self.table.setRowCount(0)
            self.total_label.setText("Всего программ: 0")
            self.size_label.setText("Общий размер: 0 КБ")
            self.info_text.clear()
            return
        
        # .robot файлы
        robot_files = sorted(self.programs_dir.glob("*.robot"))
        
        for file_path in robot_files:
            try:
                file_size = file_path.stat().st_size
                
                if file_size == 0:
                    continue
                
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read().strip()
                
                if not content:
                    continue
                
                data = json.loads(content)
                
                program_info = {
                    'name': data.get('name', file_path.stem),
                    'created': data.get('created_at', data.get('created', '')),
                    'modified': data.get('modified_at', data.get('modified', '')),
                    'description': data.get('description', ''),
                    'commands': len(data.get('commands', [])),
                    'size': file_size,
                    'program_type': data.get('program_type', 'unknown'),
                    'loop_count': data.get('loop_count', 1),
                    'file_path': str(file_path),
                    'full_data': data
                }
                
                programs.append(program_info)
                total_size += file_size
                self.program_data[program_info['name']] = program_info
            
            except (json.JSONDecodeError, Exception):
                continue
        
        # Обновить
        self.table.setRowCount(len(programs))
        
        for row, prog in enumerate(programs):
            try:
                created_dt = datetime.fromisoformat(prog['created']).strftime("%d.%m.%Y %H:%M")
            except (ValueError, TypeError):
                created_dt = "—"
            
            name_item = QTableWidgetItem(prog['name'])
            name_item.setFont(QFont("Arial", 10, QFont.Weight.Bold))
            self.table.setItem(row, 0, name_item)
            
            date_item = QTableWidgetItem(created_dt)
            self.table.setItem(row, 1, date_item)
            
            cmd_item = QTableWidgetItem(str(prog['commands']))
            cmd_item.setFont(QFont("Courier", 10, QFont.Weight.Bold))
            self.table.setItem(row, 2, cmd_item)
            
            desc_text = prog['description'][:60] + "..." if len(prog['description']) > 60 else prog['description']
            desc_item = QTableWidgetItem(desc_text)
            self.table.setItem(row, 3, desc_item)
            
            size_item = QTableWidgetItem(f"{prog['size']:,}")
            self.table.setItem(row, 4, size_item)
        
        self.total_label.setText(f"Всего программ: {len(programs)}")
        size_kb = total_size / 1024
        self.size_label.setText(f"Общий размер: {size_kb:.2f} КБ")
        self.info_text.clear()
    
    def auto_refresh(self):
        """Автоматическое обновление"""
        current_count = self.table.rowCount()
        robot_count = len(list(self.programs_dir.glob("*.robot")))
        
        if current_count != robot_count:
            self.refresh_programs()
    
    def on_program_selected(self):
        """Обработка выбора программы"""
        row = self.table.currentRow()
        if row < 0:
            self.info_text.clear()
            self.description_input.clear()
            self.save_status.setText("")
            return
        
        program_name = self.table.item(row, 0).text()
        self.current_program = program_name
        
        if program_name not in self.program_data:
            return
        
        prog = self.program_data[program_name]
        
        # Показываем описание в поле редактирования
        self.description_input.setText(prog['description'])
        self.save_status.setText("")
        
        # Показываем информацию
        info_text = f"📋 Программа: <b>{prog['name']}</b>\n\n"
        info_text += f"Тип: {prog.get('program_type', '?')}\n"
        
        try:
            created_dt = datetime.fromisoformat(prog['created']).strftime("%d.%m.%Y в %H:%M:%S")
        except:
            created_dt = "неизвестно"
        
        info_text += f"Создана: {created_dt}\n"
        info_text += f"Команд: {prog['commands']}\n"
        info_text += f"Циклов: {prog.get('loop_count', 1)}\n"
        info_text += f"Размер: {prog['size']} байт ({prog['size']/1024:.2f} КБ)"
        
        self.info_text.setHtml(info_text)
        self.program_selected.emit(program_name)
    
    def save_description(self):
        """✅ Сохранить описание в JSON файл"""
        if not self.current_program:
            QMessageBox.warning(self, "Ошибка", "Нет выбранной программы!")
            return

        try:
            file_path = self.programs_dir / f"{self.current_program}.robot"
            
            # Читаем текущее содержимое
            with open(file_path, "r", encoding="utf-8") as f:
                data = json.load(f)

            # Обновляем описание
            description = self.description_input.text()
            data["description"] = description

            # Сохраняем обратно
            with open(file_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)

            # Обновляем статус и программ_data
            self.program_data[self.current_program]['description'] = description
            self.save_status.setText("✅ Описание сохранено")
            
            # Обновляем таблицу
            self.refresh_programs()
            
            # Через 2 сек сбросим статус
            QTimer.singleShot(2000, lambda: self.save_status.setText(""))

        except Exception as e:
            QMessageBox.critical(self, "Ошибка", f"Ошибка сохранения описания: {e}")
            self.save_status.setText("❌ Ошибка сохранения")
    
    def on_delete_program(self):
        """Удалить выбранную программу"""
        if not self.current_program:
            QMessageBox.warning(self, "⚠️ Внимание", "Выберите программу для удаления!")
            return
        
        program_name = self.current_program
        
        reply = QMessageBox.question(
            self, 
            "🗑️ Удаление программы",
            f"Вы уверены, что хотите удалить программу\n'{program_name}'?\n\nЭто действие невозможно отменить!",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            try:
                file_path = self.programs_dir / f"{program_name}.robot"
                
                if file_path.exists():
                    file_path.unlink()
                    QMessageBox.information(
                        self, 
                        "✅ Успех", 
                        f"Программа '{program_name}' успешно удалена"
                    )
                    self.program_deleted.emit(program_name)
                    self.current_program = None
                    self.description_input.clear()
                    self.refresh_programs()
                else:
                    QMessageBox.warning(
                        self, 
                        "⚠️ Ошибка", 
                        f"Файл программы не найден: {file_path}"
                    )
            
            except Exception as e:
                QMessageBox.critical(
                    self, 
                    "❌ Ошибка удаления", 
                    f"Не удалось удалить программу:\n{str(e)}"
                )
    
    def on_export_program(self):
        """Экспортировать программу"""
        if not self.current_program:
            QMessageBox.warning(self, "⚠️ Внимание", "Выберите программу для экспорта!")
            return
        
        program_name = self.current_program
        source_file = self.programs_dir / f"{program_name}.robot"
        
        if not source_file.exists():
            QMessageBox.warning(self, "⚠️ Ошибка", "Файл программы не найден!")
            return
        
        file_dialog = QFileDialog()
        file_dialog.setDefaultSuffix("robot")
        file_dialog.setNameFilter("Robot Files (*.robot);;JSON Files (*.json);;All Files (*)")
        
        export_path, _ = file_dialog.getSaveFileName(
            self,
            "Экспортировать программу",
            f"{program_name}.robot",
            "Robot Files (*.robot);;JSON Files (*.json)"
        )
        
        if export_path:
            try:
                shutil.copy2(source_file, export_path)
                QMessageBox.information(
                    self,
                    "✅ Успех",
                    f"Программа экспортирована в:\n{export_path}"
                )
                self.program_exported.emit(program_name)
            
            except Exception as e:
                QMessageBox.critical(
                    self,
                    "❌ Ошибка экспорта",
                    f"Не удалось экспортировать программу:\n{str(e)}"
                )
    
    def on_import_program(self):
        """Импортировать программу"""
        file_dialog = QFileDialog()
        file_dialog.setNameFilter("Robot Files (*.robot);;JSON Files (*.json);;All Files (*)")
        
        import_path, _ = file_dialog.getOpenFileName(
            self,
            "Импортировать программу",
            "",
            "Robot Files (*.robot);;JSON Files (*.json)"
        )
        
        if import_path:
            try:
                with open(import_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                
                if 'name' not in data or 'commands' not in data:
                    QMessageBox.warning(
                        self,
                        "⚠️ Ошибка",
                        "Файл не содержит необходимые поля (name, commands)"
                    )
                    return
                
                import_file_name = Path(import_path).stem + ".robot"
                dest_path = self.programs_dir / import_file_name
                
                if dest_path.exists():
                    stem = dest_path.stem
                    counter = 1
                    while dest_path.exists():
                        dest_path = self.programs_dir / f"{stem}_{counter}.robot"
                        counter += 1
                
                shutil.copy2(import_path, dest_path)
                
                QMessageBox.information(
                    self,
                    "✅ Успех",
                    f"Программа '{data['name']}' успешно импортирована"
                )
                self.refresh_programs()
            
            except json.JSONDecodeError:
                QMessageBox.critical(
                    self,
                    "❌ Ошибка",
                    "Выбранный файл не является корректным JSON"
                )
            
            except Exception as e:
                QMessageBox.critical(
                    self,
                    "❌ Ошибка импорта",
                    f"Не удалось импортировать программу:\n{str(e)}"
                )
    
    def on_open_folder(self):
        """Открыть папку с программами"""
        try:
            import subprocess
            import sys
            
            if sys.platform == 'win32':
                os.startfile(str(self.programs_dir))
            elif sys.platform == 'darwin':
                subprocess.run(['open', str(self.programs_dir)])
            else:
                subprocess.run(['xdg-open', str(self.programs_dir)])
        
        except Exception as e:
            QMessageBox.critical(
                self,
                "❌ Ошибка",
                f"Не удалось открыть папку:\n{str(e)}"
            )
    
    def get_current_program(self):
        """Получить текущую выбранную программу"""
        if self.current_program and self.current_program in self.program_data:
            return self.program_data[self.current_program]['full_data']
        return None
    
    def get_all_programs(self):
        """Получить список всех программ"""
        return list(self.program_data.keys())
    
    def load_program(self, program_name: str):
        """Загрузить конкретную программу по имени"""
        if program_name in self.program_data:
            return self.program_data[program_name]['full_data']
        return None
    
    def get_programs_dir(self):
        """Получить путь к директории программ"""
        return str(self.programs_dir)
    
    def closeEvent(self, event):
        """При закрытии виджета"""
        self.refresh_timer.stop()
        event.accept()
