from PyQt6.QtWidgets import *
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QFont
from typing import Dict
import math


class ManualControlUI(QWidget):
  
    def __init__(self, robot_bridge):
        super().__init__()
        self.robot = robot_bridge
        
        self.speed_levels = [
            {"label": "0.1 м/c", "scale": 10.0},
            {"label": "0.01 м/c", "scale": 1.0},
            {"label": "0.001 м/c", "scale": 0.1},
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
    
    def update_speed(self):
        self.robot.speed_scale = self.speed_levels[self.speed_idx]["scale"]
    
    def init_ui(self):
        main_layout = QVBoxLayout(self)
        
        title = QLabel("РУЧНОЕ УПРАВЛЕНИЕ")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setStyleSheet("color: #0078d4; margin-bottom: 15px;")
        main_layout.addWidget(title)
        
        content = QHBoxLayout()
        
        left_panel = QVBoxLayout()
        
        linear_group = QGroupBox("ЛИНЕЙНОЕ ДВИЖЕНИЕ")
        linear_layout = QGridLayout(linear_group)
        
        # Z+ 
        up_btn = QPushButton("Z+")
        up_btn.setFixedSize(90, 60)
        up_btn.setStyleSheet(self.get_btn_style("#0078d4"))
        up_btn.pressed.connect(lambda: self.start_move(0.0, 0.0, +1.0))
        up_btn.released.connect(self.stop_move)
        linear_layout.addWidget(up_btn, 0, 1)
        
        # Z- 
        down_btn = QPushButton("Z-")
        down_btn.setFixedSize(90, 60)
        down_btn.setStyleSheet(self.get_btn_style("#0078d4"))
        down_btn.pressed.connect(lambda: self.start_move(0.0, 0.0, -1.0))
        down_btn.released.connect(self.stop_move)
        linear_layout.addWidget(down_btn, 1, 1)
        
        # Y+
        left_btn = QPushButton("Y+")
        left_btn.setFixedSize(90, 60)
        left_btn.setStyleSheet(self.get_btn_style("#28a745"))
        left_btn.pressed.connect(lambda: self.start_move(0.0, -1.0, 0.0))
        left_btn.released.connect(self.stop_move)
        linear_layout.addWidget(left_btn, 1, 2)
        
        # X+
        back_btn = QPushButton("X+")
        back_btn.setFixedSize(90, 60)
        back_btn.setStyleSheet(self.get_btn_style("#dc3545"))
        back_btn.pressed.connect(lambda: self.start_move(-1.0, 0.0, 0.0))
        back_btn.released.connect(self.stop_move)
        linear_layout.addWidget(back_btn, 2, 1)
        
        # X-
        forward_btn = QPushButton("X-")
        forward_btn.setFixedSize(90, 60)
        forward_btn.setStyleSheet(self.get_btn_style("#dc3545"))
        forward_btn.pressed.connect(lambda: self.start_move(+1.0, 0.0, 0.0))
        forward_btn.released.connect(self.stop_move)
        linear_layout.addWidget(forward_btn, 3, 1)
        
        # Y-
        right_btn = QPushButton("Y-")
        right_btn.setFixedSize(90, 60)
        right_btn.setStyleSheet(self.get_btn_style("#28a745"))
        right_btn.pressed.connect(lambda: self.start_move(0.0, +1.0, 0.0))
        right_btn.released.connect(self.stop_move)
        linear_layout.addWidget(right_btn, 1, 0)
        
       
        
        left_panel.addWidget(linear_group)
        

        angular_group = QGroupBox("УГЛОВОЙ ПОВОРОТ")
        angular_layout = QGridLayout(angular_group)
        
        # RX+
        rx_plus = QPushButton("RX-")
        rx_plus.setFixedSize(90, 50)
        rx_plus.setStyleSheet(self.get_btn_style("#ffc107"))
        rx_plus.pressed.connect(lambda: self.start_rotate(+1.0, 0.0, 0.0))
        rx_plus.released.connect(self.stop_rotate)
        angular_layout.addWidget(rx_plus, 0, 0)
        
        # RX-
        rx_minus = QPushButton("RX+")
        rx_minus.setFixedSize(90, 50)
        rx_minus.setStyleSheet(self.get_btn_style("#ffc107"))
        rx_minus.pressed.connect(lambda: self.start_rotate(-1.0, 0.0, 0.0))
        rx_minus.released.connect(self.stop_rotate)
        angular_layout.addWidget(rx_minus, 0, 1)
        
        # RY-
        ry_plus = QPushButton("RY-")
        ry_plus.setFixedSize(90, 50)
        ry_plus.setStyleSheet(self.get_btn_style("#ffc107"))
        ry_plus.pressed.connect(lambda: self.start_rotate(0.0, +1.0, 0.0))
        ry_plus.released.connect(self.stop_rotate)
        angular_layout.addWidget(ry_plus, 1, 0)
        
        # RY+
        ry_minus = QPushButton("RY+")
        ry_minus.setFixedSize(90, 50)
        ry_minus.setStyleSheet(self.get_btn_style("#ffc107"))
        ry_minus.pressed.connect(lambda: self.start_rotate(0.0, -1.0, 0.0))
        ry_minus.released.connect(self.stop_rotate)
        angular_layout.addWidget(ry_minus, 1, 1)
        
        # RZ-
        rz_plus = QPushButton("RZ-")
        rz_plus.setFixedSize(90, 50)
        rz_plus.setStyleSheet(self.get_btn_style("#17a2b8"))
        rz_plus.pressed.connect(lambda: self.start_rotate(0.0, 0.0, +1.0))
        rz_plus.released.connect(self.stop_rotate)
        angular_layout.addWidget(rz_plus, 2, 0)
        
        # RZ+
        rz_minus = QPushButton("RZ+")
        rz_minus.setFixedSize(90, 50)
        rz_minus.setStyleSheet(self.get_btn_style("#17a2b8"))
        rz_minus.pressed.connect(lambda: self.start_rotate(0.0, 0.0, -1.0))
        rz_minus.released.connect(self.stop_rotate)
        angular_layout.addWidget(rz_minus, 2, 1)
        
        left_panel.addWidget(angular_group)
        left_panel.addStretch()
        
        left_group = QGroupBox()
        left_group.setLayout(left_panel)
        content.addWidget(left_group)
        
        # ==================== ПРАВАЯ ПАНЕЛЬ ====================
        right_group = QGroupBox("СТАТУС И УПРАВЛЕНИЕ")
        right_layout = QVBoxLayout()
        
        # Текущие углы суставов
        status_title = QLabel("ТЕКУЩИЕ УГЛЫ СУСТАВОВ:")
        status_title.setStyleSheet(
            "font-weight: bold; color: #0078d4; font-size: 11px;"
        )
        right_layout.addWidget(status_title)
        
        self.position_display = QLabel(
            "J1: 0.0°  J2: 0.0°  J3: 0.0°\n"
            "J4: 0.0°  J5: 0.0°  J6: 0.0°"
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
        
        # Поза TCP (мировые оси)
        pose_label = QLabel("КООРДИНАТЫ")
        pose_label.setStyleSheet(
            "font-weight: bold; color: #0078d4; font-size: 11px; margin-top: 10px;"
        )
        right_layout.addWidget(pose_label)
        
        self.pose_display = QLabel(
            "X: --- mm  Y: --- mm  Z: --- mm\n"
            "RX: ---°  RY: ---°  RZ: ---°"
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
        
        # ✅ v3.0: Выбор скорости
        speed_label = QLabel("СКОРОСТЬ ПОДАЧИ:")
        speed_label.setStyleSheet(
            "font-weight: bold; color: #ff6600; font-size: 11px; margin-top: 10px;"
        )
        right_layout.addWidget(speed_label)
        
        self.speed_combo = QComboBox()
        for level in self.speed_levels:
            self.speed_combo.addItem(level["label"])
        self.speed_combo.setCurrentIndex(1)
        self.speed_combo.currentIndexChanged.connect(self.on_speed_changed)
        self.speed_combo.setStyleSheet(
            "QComboBox { "
            "    padding: 5px; "
            "    border-radius: 4px; "
            "    background-color: #333; "
            "    color: white; "
            "    border: 1px solid #555; "
            "} "
            "QComboBox::drop-down { "
            "    border: none; "
            "} "
            "QComboBox::down-arrow { "
            "    image: none; "
            "}"
        )
        right_layout.addWidget(self.speed_combo)
        
        # Кнопки управления
        btn_layout = QHBoxLayout()
        
        home_btn = QPushButton("ИСХОДНАЯ ПОЗИЦИЯ")
        home_btn.setStyleSheet(self.get_btn_style("#28a745"))
        home_btn.clicked.connect(self.on_home)
        btn_layout.addWidget(home_btn)
        
        right_layout.addLayout(btn_layout)
        
        right_layout.addStretch()
        right_group.setLayout(right_layout)
        content.addWidget(right_group)
        
        main_layout.addLayout(content)
        main_layout.addStretch()
        self.setLayout(main_layout)
    
    def get_btn_style(self, color: str) -> str:
        """
        ✅ ИСПРАВЛЕННЫЙ CSS для PyQt6 (БЕЗ box-shadow)
        """
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
    
    def on_speed_changed(self, index: int):
        """Обработчик изменения скорости."""
        self.speed_idx = index
        self.update_speed()
        self.speed_combo.blockSignals(True)
        self.speed_combo.setCurrentIndex(index)
        self.speed_combo.blockSignals(False)
    
    def start_move(self, dx: float, dy: float, dz: float):
        """Начинает линейное движение."""
        length = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
        if length == 0:
            return
        
        # Нормализуем
        k = 1.0 / length
        self.current_move = (dx * k, dy * k, dz * k)
        
        if not self.move_timer.isActive():
            self.move_timer.start()
    
    def stop_move(self):
        """Останавливает линейное движение."""
        self.current_move = (0.0, 0.0, 0.0)
        self.move_timer.stop()
    
    def start_rotate(self, drx: float, dry: float, drz: float):
        """Начинает угловое движение."""
        length = (drx**2 + dry**2 + drz**2) ** 0.5
        if length == 0:
            return
        
        # Нормализуем
        k = 1.0 / length
        self.current_drot = (drx * k, dry * k, drz * k)
        
        if not self.rotate_timer.isActive():
            self.rotate_timer.start()
    
    def stop_rotate(self):
        """Останавливает угловое движение."""
        self.current_drot = (0.0, 0.0, 0.0)
        self.rotate_timer.stop()
    
    def on_home(self):
        """Возврат в исходную позицию."""
        self.robot.reset_position()

    
    def on_move_timer(self):
        """Таймер для линейного движения."""
        dx, dy, dz = self.current_move
        
        if dx == dy == dz == 0.0:
            return
        
        # Применяем масштаб скорости
        scaled_step_size = 0.01 * self.robot.speed_scale
        
        self.robot.move_end_effector_world(
            dx * scaled_step_size,
            dy * scaled_step_size,
            dz * scaled_step_size,
            duration=0.05
        )
    
    def on_rotate_timer(self):
        """Таймер для углового движения."""
        drx, dry, drz = self.current_drot
        
        if drx == dry == drz == 0.0:
            return
        
        # ✅ v3.0: Применяем угловой шаг
        angle_delta = self.angle_step
        
        # RX и RY через ИК (БЕЗ дрейфа)
        if drx != 0.0 or dry != 0.0:
            self.robot.rotate_end_effector_rx_ry_ik(
                drx * angle_delta,
                dry * angle_delta,
                duration=0.05
            )
        
        # RZ через прямое управление wrist_3
        if drz != 0.0:
            self.robot.rotate_end_effector_world(
                drz * angle_delta,
                duration=0.05
            )
    
    def on_ros_update(self, data: Dict):
        """Обновляет отображение из ROS callback."""
        if 'position' in data:
            positions = data['position']
            angles_deg = [p * (180.0 / math.pi) for p in positions]
            
            status_joints = ""
            for i, angle in enumerate(angles_deg, 1):
                status_joints += f"J{i}: {angle:6.1f}° "
                if i % 3 == 0:
                    status_joints += "\n"
            
            self.position_display.setText(status_joints.strip())
        
        if 'pose' in data:
            pose = data['pose']
            self.pose_display.setText(
                f"X: {pose['x']*1000:7.1f} mm  "
                f"Y: {pose['y']*1000:7.1f} mm  "
                f"Z: {pose['z']*1000:7.1f} mm\n"
                f"RX: {pose['rx']:7.1f}°  "
                f"RY: {pose['ry']:7.1f}°  "
                f"RZ: {pose['rz']:7.1f}°"
            )
