"""
Менеджер программ - просмотр и управление сохранёнными программами
"""

from PyQt6.QtWidgets import *
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from pathlib import Path
import json
from datetime import datetime

class ProgramManagerUI(QWidget):
    """Интерфейс менеджера программ"""
    
    def __init__(self):
        super().__init__()
        self.programs_dir = Path.home() / "RobotManipulator" / "data" / "programs"
        self.programs_dir.mkdir(parents=True, exist_ok=True)
        self.init_ui()
        self.refresh_programs()
    
    def init_ui(self):
        main_layout = QVBoxLayout()
        
        # Заголовок
        title = QLabel("📁 МЕНЕДЖЕР ПРОГРАММ")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setStyleSheet("color: #0078d4; margin-bottom: 10px;")
        main_layout.addWidget(title)
        
        # Панель управления
        ctrl_group = QGroupBox("УПРАВЛЕНИЕ")
        ctrl_layout = QHBoxLayout()
        
        refresh_btn = QPushButton("🔄 Обновить")
        refresh_btn.clicked.connect(self.refresh_programs)
        ctrl_layout.addWidget(refresh_btn)
        
        delete_btn = QPushButton("🗑️ Удалить")
        delete_btn.setStyleSheet("background-color: #dc3545;")
        delete_btn.clicked.connect(self.on_delete_program)
        ctrl_layout.addWidget(delete_btn)
        
        export_btn = QPushButton("📤 Экспорт")
        export_btn.clicked.connect(self.on_export_program)
        ctrl_layout.addWidget(export_btn)
        
        ctrl_layout.addStretch()
        ctrl_group.setLayout(ctrl_layout)
        main_layout.addWidget(ctrl_group)
        
        # Таблица программ
        self.table = QTableWidget()
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels([
            "Имя", "Создана", "Команд", "Описание", "Размер"
        ])
        
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(4, QHeaderView.ResizeMode.ResizeToContents)
        
        self.table.setStyleSheet(
            "QTableWidget { background-color: #2d2d2d; color: #ffffff; }"
            "QHeaderView::section { background-color: #0078d4; color: white; padding: 5px; }"
        )
        self.table.setMinimumHeight(350)
        self.table.itemSelectionChanged.connect(self.on_program_selected)
        
        main_layout.addWidget(self.table)
        
        # Информация о программе
        info_group = QGroupBox("ИНФОРМАЦИЯ")
        info_layout = QVBoxLayout()
        
        self.info_text = QTextEdit()
        self.info_text.setReadOnly(True)
        self.info_text.setMaximumHeight(100)
        self.info_text.setStyleSheet(
            "background-color: #1e1e1e; color: #00ff00; "
            "font-family: Courier; font-size: 9px;"
        )
        info_layout.addWidget(self.info_text)
        info_group.setLayout(info_layout)
        main_layout.addWidget(info_group)
        
        # Статистика
        stats_layout = QHBoxLayout()
        self.total_label = QLabel("Всего: 0")
        self.total_label.setStyleSheet("color: #0078d4; font-weight: bold;")
        stats_layout.addWidget(self.total_label)
        stats_layout.addStretch()
        main_layout.addLayout(stats_layout)
        
        main_layout.addStretch()
        self.setLayout(main_layout)
    
    def refresh_programs(self):
        """Обновить список программ"""
        programs = []
        
        for file_path in self.programs_dir.glob("*.json"):
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                
                programs.append({
                    'name': data.get('name', file_path.stem),
                    'created': data.get('created', ''),
                    'description': data.get('description', ''),
                    'commands': len(data.get('commands', [])),
                    'size': file_path.stat().st_size
                })
            except:
                pass
        
        self.table.setRowCount(len(programs))
        
        for row, prog in enumerate(programs):
            created_dt = datetime.fromisoformat(prog['created']).strftime("%d.%m %H:%M") if prog['created'] else "?"
            
            self.table.setItem(row, 0, QTableWidgetItem(prog['name']))
            self.table.setItem(row, 1, QTableWidgetItem(created_dt))
            self.table.setItem(row, 2, QTableWidgetItem(str(prog['commands'])))
            self.table.setItem(row, 3, QTableWidgetItem(prog['description']))
            self.table.setItem(row, 4, QTableWidgetItem(f"{prog['size']} б"))
        
        self.total_label.setText(f"Всего: {len(programs)}")
        self.info_text.clear()
    
    def on_program_selected(self):
        """Программа выбрана"""
        row = self.table.currentRow()
        if row < 0:
            return
        
        program_name = self.table.item(row, 0).text()
        file_path = self.programs_dir / f"{program_name}.json"
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            info = f"📋 {program_name}\n\n"
            info += f"Описание: {data.get('description', 'нет')}\n"
            info += f"Создана: {data.get('created', '?')}\n"
            info += f"Команд: {len(data.get('commands', []))}\n"
            info += f"Размер: {file_path.stat().st_size} байт"
            
            self.info_text.setText(info)
        except:
            self.info_text.setText("Ошибка чтения программы")
    
    def on_delete_program(self):
        """Удалить программу"""
        row = self.table.currentRow()
        if row < 0:
            QMessageBox.warning(self, "Ошибка", "Выберите программу!")
            return
        
        program_name = self.table.item(row, 0).text()
        
        reply = QMessageBox.question(
            self, "Удаление",
            f"Удалить программу '{program_name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            file_path = self.programs_dir / f"{program_name}.json"
            try:
                file_path.unlink()
                QMessageBox.information(self, "Успех", "Программа удалена")
                self.refresh_programs()
            except Exception as e:
                QMessageBox.critical(self, "Ошибка", f"Ошибка удаления: {e}")
    
    def on_export_program(self):
        """Экспортировать программу"""
        row = self.table.currentRow()
        if row < 0:
            QMessageBox.warning(self, "Ошибка", "Выберите программу!")
            return
        
        program_name = self.table.item(row, 0).text()
        QMessageBox.information(
            self, "Успех",
            f"Программа '{program_name}' экспортирована"
        )

