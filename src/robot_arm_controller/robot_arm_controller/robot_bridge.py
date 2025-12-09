"""
🤖 ROBOT BRIDGE v3.0 - ПОЛНОЕ РЕШЕНИЕ

✅ ПОЛНЫЕ УЛУЧШЕНИЯ:
  ✓ 3 варианта скорости (0.1м, 0.01м, 0.001м)
  ✓ Правильное RX/RY вращение (БЕЗ линейного дрейфа)
  ✓ Фиксированный центр хвата при вращении
  ✓ Оптимальное демпфирование для каждого режима
  ✓ Активная компенсация позиции при вращении

АРХИТЕКТУРА:
1. move_end_effector_world() - линейное движение с масштабом скорости
2. rotate_end_effector_rx_ry_ik() - вращение БЕЗ дрейфа позиции
3. rotate_end_effector_world() - вращение вокруг RZ
4. ik_step_6d() - дифференциальный 6D ИК с DLS демпфированием

ИСПОЛЬЗОВАНИЕ:
  robot.speed_scale = 10.0   # 0.1м - БЫСТРО
  robot.speed_scale = 1.0    # 0.01м - НОРМАЛЬНО
  robot.speed_scale = 0.1    # 0.001м - МЕДЛЕННО
"""

import math
import threading
from typing import List, Dict, Callable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class RobotBridge(Node):
    """
    Мост между приложением и UR5 через ROS 2.
    
    ✅ v3.0: 3 режима скорости + правильное RX/RY вращение
    """
    
    def __init__(self):
        super().__init__('robot_arm_controller')
        
        self.current_position: List[float] = [0.0] * 6
        self.is_connected: bool = False
        self.callbacks: List[Callable[[Dict], None]] = []
        self.lock = threading.Lock()
        
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        self.state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.ros_thread = threading.Thread(target=self.spin_thread, daemon=True)
        self.ros_thread.start()
        
        self.is_connected = True
        self.get_logger().info("✅ Мост ROS 2 инициализирован v3.0")
        
        self.ref_R: List[List[float]] | None = None
        self.ref_pose_valid: bool = False
        
        # ✅ НОВОЕ v3.0: Хранилище для скоростей
        self.speed_scale = 1.0  # 1.0 = 0.01м, 0.1 = 0.001м, 10.0 = 0.1м

    # ================================================================ #
    # БАЗОВЫЕ ROS-ОБРАБОТЧИКИ
    # ================================================================ #
    
    def spin_thread(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
    
    def timer_callback(self):
        pass
    
    def joint_state_callback(self, msg: JointState):
        ur_joints = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        
        positions: List[float] = []
        for joint_name in ur_joints:
            try:
                idx = msg.name.index(joint_name)
                positions.append(msg.position[idx])
            except ValueError:
                positions.append(0.0)
        
        with self.lock:
            self.current_position = positions
        
        pose = self.get_tcp_pose()
        self.notify_callbacks({'position': positions, 'pose': pose})

    # ================================================================ #
    # ПРЯМАЯ КИНЕМАТИКА (ДХ параметры UR5)
    # ================================================================ #
    
    def forward_kinematics(self, joints: List[float]):
        """Вычисляет позицию TCP по суставам."""
        d = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
        a = [0.0, -0.425, -0.39225, 0.0, 0.0, 0.0]
        alpha = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]
        
        def dh(a_i, alpha_i, d_i, theta_i):
            ca, sa = math.cos(alpha_i), math.sin(alpha_i)
            ct, st = math.cos(theta_i), math.sin(theta_i)
            return [
                [ct, -st * ca, st * sa, a_i * ct],
                [st, ct * ca, -ct * sa, a_i * st],
                [0.0, sa, ca, d_i],
                [0.0, 0.0, 0.0, 1.0],
            ]
        
        T = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        
        def matmul(A, B):
            res = [[0.0] * 4 for _ in range(4)]
            for i in range(4):
                for j in range(4):
                    res[i][j] = sum(A[i][k] * B[k][j] for k in range(4))
            return res
        
        for i in range(6):
            Ti = dh(a[i], alpha[i], d[i], joints[i])
            T = matmul(T, Ti)
        
        return T[0][3], T[1][3], T[2][3]
    
    def forward_kinematics_full(self, joints: List[float]):
        """Вычисляет позицию И ориентацию TCP."""
        d = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
        a = [0.0, -0.425, -0.39225, 0.0, 0.0, 0.0]
        alpha = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]
        
        def dh(a_i, alpha_i, d_i, theta_i):
            ca, sa = math.cos(alpha_i), math.sin(alpha_i)
            ct, st = math.cos(theta_i), math.sin(theta_i)
            return [
                [ct, -st * ca, st * sa, a_i * ct],
                [st, ct * ca, -ct * sa, a_i * st],
                [0.0, sa, ca, d_i],
                [0.0, 0.0, 0.0, 1.0],
            ]
        
        T = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        
        def matmul(A, B):
            res = [[0.0] * 4 for _ in range(4)]
            for i in range(4):
                for j in range(4):
                    res[i][j] = sum(A[i][k] * B[k][j] for k in range(4))
            return res
        
        for i in range(6):
            Ti = dh(a[i], alpha[i], d[i], joints[i])
            T = matmul(T, Ti)
        
        R = [
            [T[0][0], T[0][1], T[0][2]],
            [T[1][0], T[1][1], T[1][2]],
            [T[2][0], T[2][1], T[2][2]],
        ]
        
        p = [T[0][3], T[1][3], T[2][3]]
        return R, p

    # ================================================================ #
    # ВСПОМОГАТЕЛЬНАЯ МАТЕМАТИКА
    # ================================================================ #
    
    @staticmethod
    def _mat3_vec3(R, v):
        return [
            R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
        ]
    
    @staticmethod
    def _so3_log(R):
        """Логарифм SO(3): матрица → вектор угловой скорости."""
        tr = R[0][0] + R[1][1] + R[2][2]
        cos_theta = max(min((tr - 1.0) / 2.0, 1.0), -1.0)
        theta = math.acos(cos_theta)
        
        if abs(theta) < 1e-6:
            return [0.0, 0.0, 0.0]
        
        sin_theta = math.sin(theta)
        k = theta / (2.0 * sin_theta)
        
        wx = k * (R[2][1] - R[1][2])
        wy = k * (R[0][2] - R[2][0])
        wz = k * (R[1][0] - R[0][1])
        
        return [wx, wy, wz]

    # ================================================================ #
    # КВАТЕРНИОНЫ
    # ================================================================ #
    
    @staticmethod
    def _quat_normalize(q):
        x, y, z, w = q
        n = math.sqrt(x*x + y*y + z*z + w*w)
        if n < 1e-12:
            return [0.0, 0.0, 0.0, 1.0]
        return [x / n, y / n, z / n, w / n]
    
    @staticmethod
    def _quat_mul(q1, q2):
        """q = q1 * q2 (первым применяется q2, затем q1)."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        
        return RobotBridge._quat_normalize([x, y, z, w])
    
    @staticmethod
    def _axis_angle_to_quat(axis, angle):
        """axis: [ax, ay, az] (единичный), angle: радианы."""
        ax, ay, az = axis
        half = angle * 0.5
        s = math.sin(half)
        return RobotBridge._quat_normalize([ax * s, ay * s, az * s, math.cos(half)])
    
    @staticmethod
    def _rot_to_quat(R):
        """3x3 → кватернион [x,y,z,w]."""
        tr = R[0][0] + R[1][1] + R[2][2]
        
        if tr > 0.0:
            S = math.sqrt(tr + 1.0) * 2.0
            w = 0.25 * S
            x = (R[2][1] - R[1][2]) / S
            y = (R[0][2] - R[2][0]) / S
            z = (R[1][0] - R[0][1]) / S
        elif (R[0][0] > R[1][1]) and (R[0][0] > R[2][2]):
            S = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
            w = (R[2][1] - R[1][2]) / S
            x = 0.25 * S
            y = (R[0][1] + R[1][0]) / S
            z = (R[0][2] + R[2][0]) / S
        elif R[1][1] > R[2][2]:
            S = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
            w = (R[0][2] - R[2][0]) / S
            x = (R[0][1] + R[1][0]) / S
            y = 0.25 * S
            z = (R[1][2] + R[2][1]) / S
        else:
            S = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
            w = (R[1][0] - R[0][1]) / S
            x = (R[0][2] + R[2][0]) / S
            y = (R[1][2] + R[2][1]) / S
            z = 0.25 * S
        
        return RobotBridge._quat_normalize([x, y, z, w])
    
    @staticmethod
    def _quat_to_rot(q):
        """Кватернион [x,y,z,w] → 3x3 матрица R."""
        x, y, z, w = RobotBridge._quat_normalize(q)
        
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        
        return [
            [1.0 - 2.0*(yy + zz), 2.0*(xy - wz), 2.0*(xz + wy)],
            [2.0*(xy + wz), 1.0 - 2.0*(xx + zz), 2.0*(yz - wx)],
            [2.0*(xz - wy), 2.0*(yz + wx), 1.0 - 2.0*(xx + yy)],
        ]

    # ================================================================ #
    # ЯКОБИАНЫ (численный дифференциал)
    # ================================================================ #
    
    def compute_position_jacobian(self, joints: List[float]):
        """Якобиан для позиции (3x6)."""
        eps = 2e-4
        x0, y0, z0 = self.forward_kinematics(joints)
        
        J_tmp = []
        for i in range(6):
            j_pert = joints.copy()
            j_pert[i] += eps
            x1, y1, z1 = self.forward_kinematics(j_pert)
            J_tmp.append([
                (x1 - x0) / eps,
                (y1 - y0) / eps,
                (z1 - z0) / eps
            ])
        
        return [[J_tmp[j][i] for j in range(6)] for i in range(3)]
    
    def compute_orientation_jacobian(self, joints: List[float]):
        """Якобиан для ориентации (3x6)."""
        eps = 2e-4
        R0, _ = self.forward_kinematics_full(joints)
        
        J_tmp = []
        for i in range(6):
            j_pert = joints.copy()
            j_pert[i] += eps
            R1, _ = self.forward_kinematics_full(j_pert)
            
            Rt = [[R0[r][c] for c in range(3)] for r in range(3)]
            R_err = [
                [Rt[r][0] * R1[0][c] + Rt[r][1] * R1[1][c] + Rt[r][2] * R1[2][c]
                 for c in range(3)]
                for r in range(3)
            ]
            
            w = self._so3_log(R_err)
            J_tmp.append([w[0] / eps, w[1] / eps, w[2] / eps])
        
        return [[J_tmp[j][i] for j in range(6)] for i in range(3)]

    # ================================================================ #
    # 3D-ИК (только позиция)
    # ================================================================ #
    
    def ik_step(self, joints: List[float], dx: float, dy: float, dz: float,
                step_scale: float = 0.7):
        """Один шаг дифференциального ИК по позиции (3D)."""
        x0, y0, z0 = self.forward_kinematics(joints)
        J = self.compute_position_jacobian(joints)  # 3x6
        v = [dx, dy, dz]
        
        # ✅ Демпфирование для стабильности
        lam = 5e-2
        
        # J*J^T
        A = [[0.0] * 3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                A[i][j] = sum(J[i][k] * J[j][k] for k in range(6))
        
        for i in range(3):
            A[i][i] += lam * lam
        
        # Гаусс с обратной подстановкой
        w = v[:]
        n = 3
        
        for i in range(n):
            pivot = A[i][i]
            if abs(pivot) < 1e-12:
                continue
            inv_p = 1.0 / pivot
            for j in range(i, n):
                A[i][j] *= inv_p
            w[i] *= inv_p
            
            for k in range(i + 1, n):
                factor = A[k][i]
                for j in range(i, n):
                    A[k][j] -= factor * A[i][j]
                w[k] -= factor * w[i]
        
        for i in range(n - 1, -1, -1):
            for j in range(i + 1, n):
                w[i] -= A[i][j] * w[j]
        
        dtheta = [0.0] * 6
        for j in range(6):
            dtheta[j] = step_scale * (
                J[0][j] * w[0] +
                J[1][j] * w[1] +
                J[2][j] * w[2]
            )
        
        # ✅ Ограничение шага
        max_step = 0.02
        for i in range(6):
            dtheta[i] = max(-max_step, min(dtheta[i], max_step))
        
        return [j + d for j, d in zip(joints, dtheta)]

    # ================================================================ #
    # 6D-ИК (позиция + ориентация)
    # ================================================================ #
    
    def ik_step_6d(self, joints: List[float], Rd, pd,
                   step_scale_pos: float = 1.0,
                   step_scale_rot: float = 1.0,
                   lam: float = 1e-2):
        """6D дифференциальный ИК (позиция + ориентация) с DLS."""
        R, p = self.forward_kinematics_full(joints)
        
        # Позиционная ошибка
        ep = [pd[i] - p[i] for i in range(3)]
        
        # Ориентационная ошибка
        Rt = [[R[r][c] for c in range(3)] for r in range(3)]
        R_err = [
            [Rt[r][0] * Rd[0][c] + Rt[r][1] * Rd[1][c] + Rt[r][2] * Rd[2][c]
             for c in range(3)]
            for r in range(3)
        ]
        ew = self._so3_log(R_err)
        
        # Скейлируем ошибки
        ep = [step_scale_pos * v for v in ep]
        ew = [step_scale_rot * w for w in ew]
        e = ep + ew  # 6-мерный вектор
        
        # Якобианы
        J_pos = self.compute_position_jacobian(joints)  # 3x6
        J_rot = self.compute_orientation_jacobian(joints)  # 3x6
        
        J6 = J_pos + J_rot  # 6x6
        
        # DLS: dθ = J^T (J J^T + λ^2 I)^(-1) e
        JJt = [[0.0] * 6 for _ in range(6)]
        for i in range(6):
            for j in range(6):
                JJt[i][j] = sum(J6[i][k] * J6[j][k] for k in range(6))
        
        for i in range(6):
            JJt[i][i] += lam * lam
        
        # Гаусс
        w = e[:]
        n = 6
        
        for i in range(n):
            pivot = JJt[i][i]
            if abs(pivot) < 1e-12:
                continue
            inv_p = 1.0 / pivot
            for j in range(i, n):
                JJt[i][j] *= inv_p
            w[i] *= inv_p
            
            for k in range(i + 1, n):
                factor = JJt[k][i]
                if abs(factor) < 1e-14:
                    continue
                for j in range(i, n):
                    JJt[k][j] -= factor * JJt[i][j]
                w[k] -= factor * w[i]
        
        for i in range(n - 1, -1, -1):
            for j in range(i + 1, n):
                w[i] -= JJt[i][j] * w[j]
        
        # dθ = J^T w
        dtheta = [0.0] * 6
        for j in range(6):
            dtheta[j] = sum(J6[i][j] * w[i] for i in range(6))
        
        # Ограничение шага
        max_step = 0.02
        for i in range(6):
            dtheta[i] = max(-max_step, min(dtheta[i], max_step))
        
        return [joints[i] + dtheta[i] for i in range(6)]

    # ================================================================ #
    # ЛИНЕЙНОЕ ДВИЖЕНИЕ (v3.0: с поддержкой 3 скоростей)
    # ================================================================ #
    
    def move_end_effector_world(self, dx: float = 0.0, dy: float = 0.0,
                                dz: float = 0.0, duration: float = 0.8):
        """
        ✅ v3.0: Линейное движение TCP в мировых осях X,Y,Z
        
        Масштабирование скорости через self.speed_scale:
          10.0 → 0.1м (БЫСТРО)
          1.0  → 0.01м (НОРМАЛЬНО)
          0.1  → 0.001м (МЕДЛЕННО)
        """
        if abs(dx) < 1e-9 and abs(dy) < 1e-9 and abs(dz) < 1e-9:
            return
        
        # ✅ v3.0: Применяем масштаб скорости
        dx *= self.speed_scale
        dy *= self.speed_scale
        dz *= self.speed_scale
        
        with self.lock:
            q = list(self.current_position)
        
        R_target, p_start = self.forward_kinematics_full(q)
        p_target = [p_start[0] + dx, p_start[1] + dy, p_start[2] + dz]
        
        total_dist = abs(dx) + abs(dy) + abs(dz)
        steps = max(10, int(20 * total_dist))
        
        step_dx = dx / steps
        step_dy = dy / steps
        step_dz = dz / steps
        
        p_current = list(p_start)
        lam_value = 0.1
        
        for step_num in range(steps):
            p_current[0] += step_dx
            p_current[1] += step_dy
            p_current[2] += step_dz
            
            q = self.ik_step_6d(
                q,
                Rd=R_target,
                pd=p_current,
                step_scale_pos=1.0,
                step_scale_rot=0.0,
                lam=lam_value
            )
            
            # Адаптивная коррекция
            if (step_num + 1) % 5 == 0:
                _, p_tmp = self.forward_kinematics_full(q)
                dp = [p_tmp[i] - p_current[i] for i in range(3)]
                
                if abs(dp[0]) > 1e-4 or abs(dp[1]) > 1e-4 or abs(dp[2]) > 1e-4:
                    q = self.ik_step(q, -dp[0]*0.95, -dp[1]*0.95, -dp[2]*0.95,
                                    step_scale=0.95)
        
        self.send_trajectory(q, duration)

    # ================================================================ #
    # ВРАЩЕНИЕ ВОКРУГ RX/RY (v3.0: БЕЗ ЛИНЕЙНОГО ДРЕЙФА)
    # ================================================================ #
    
    def rotate_end_effector_rx_ry_ik(self,
                                     d_rx: float = 0.0,
                                     d_ry: float = 0.0,
                                     duration: float = 0.1):
        """
        ✅ v3.0: ИСПРАВЛЕННОЕ вращение вокруг RX/RY
        
        КРИТИЧНО: Позиция TCP (X,Y,Z в мире) ОСТАЕТСЯ НЕПОДВИЖНОЙ
        Допускаются ошибки только ±0.1-0.3 мм
        
        Вращение происходит БЕЗ линейного дрейфа
        через кватернионы в локальных осях TCP
        """
        if abs(d_rx) < 1e-9 and abs(d_ry) < 1e-9:
            return
        
        with self.lock:
            q_j = list(self.current_position)
        
        R_cur, p_cur = self.forward_kinematics_full(q_j)
        q_cur = self._rot_to_quat(R_cur)
        
        # Определяем направление поворота
        q_step = [0.0, 0.0, 0.0, 1.0]
        
        if abs(d_rx) > 1e-9 and abs(d_ry) < 1e-9:
            q_step = self._axis_angle_to_quat([1.0, 0.0, 0.0], d_rx)
        elif abs(d_ry) > 1e-9 and abs(d_rx) < 1e-9:
            q_step = self._axis_angle_to_quat([0.0, 1.0, 0.0], d_ry)
        else:
            # Комбинированный поворот - разбиваем на два
            self.rotate_end_effector_rx_ry_ik(d_rx=d_rx, d_ry=0.0, duration=duration)
            self.rotate_end_effector_rx_ry_ik(d_rx=0.0, d_ry=d_ry, duration=duration)
            return
        
        # Целевая ориентация
        q_target = self._quat_mul(q_cur, q_step)
        Rd = self._quat_to_rot(q_target)
        
        # ✅ v3.0: МНОГО итераций с ВЫСОКИМ демпфированием
        iters = 40
        lam_rot = 0.12
        
        for iter_num in range(iters):
            q_j = self.ik_step_6d(
                q_j, Rd, p_cur,
                step_scale_pos=0.0,  # КРИТИЧНО: позиция не меняется
                step_scale_rot=0.5,
                lam=lam_rot
            )
            
            # ✅ v3.0: АКТИВНАЯ компенсация дрейфа после каждой итерации
            _, p_tmp = self.forward_kinematics_full(q_j)
            dp = [p_tmp[i] - p_cur[i] for i in range(3)]
            
            if abs(dp[0]) > 1e-5 or abs(dp[1]) > 1e-5 or abs(dp[2]) > 1e-5:
                q_j = self.ik_step(q_j, -dp[0]*0.8, -dp[1]*0.8, -dp[2]*0.8,
                                  step_scale=0.9)
        
        self.send_trajectory(q_j, duration)

    # ================================================================ #
    # ВРАЩЕНИЕ ВОКРУГ RZ (локальная ось Z хвата)
    # ================================================================ #
    
    def rotate_end_effector_world(self, drz: float, duration: float = 0.1):
        """Прямое кручение вокруг локальной оси Z через wrist_3."""
        with self.lock:
            joints = list(self.current_position)
        
        scale = 0.5
        joints[5] += drz * scale
        
        self.send_trajectory(joints, duration)

    # ================================================================ #
    # СЛУЖЕБНЫЕ ФУНКЦИИ
    # ================================================================ #
    
    def send_trajectory(self, joint_angles: List[float], duration_sec: float = 1.0):
        """Отправляет траекторию на робот."""
        traj = JointTrajectory()
        traj.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        traj.points.append(point)
        
        self.trajectory_publisher.publish(traj)
    
    def move_joint(self, joint_index: int, angle: float, duration_sec: float = 0.5):
        """Движение одного сустава на абсолютный угол."""
        with self.lock:
            new_position = list(self.current_position)
            new_position[joint_index] = angle
            self.send_trajectory(new_position, duration_sec)
    
    def move_joints_absolute(self, angles: List[float], duration_sec: float = 1.0):
        """Движение всех суставов на абсолютные углы."""
        self.send_trajectory(angles, duration_sec)
    
    def reset_position(self):
        """Возврат в исходную позицию."""
        home = [
            0.0,
            -math.pi / 2.0,
            math.pi / 2.0,
            0.0,
            0.0,
            0.0,
        ]
        self.send_trajectory(home, 2.0)
    
    def get_current_position(self) -> List[float]:
        """Текущие углы суставов."""
        with self.lock:
            return self.current_position.copy()
    
    def register_callback(self, callback: Callable[[Dict], None]):
        """Регистрирует callback для обновлений."""
        self.callbacks.append(callback)
    
    def notify_callbacks(self, data: Dict):
        """Уведомляет все callbacks."""
        for callback in self.callbacks:
            try:
                callback(data)
            except Exception as e:
                self.get_logger().error(f"Ошибка в callback: {e}")
    
    def get_tcp_pose(self) -> Dict[str, float]:
        """Вычисляет текущую позу TCP: x,y,z,rx,ry,rz."""
        with self.lock:
            q = list(self.current_position)
        
        x, y, z = self.forward_kinematics(q)
        R, _ = self.forward_kinematics_full(q)
        
        # Эйлер ZYX (RZ,RY,RX)
        sy = math.sqrt(R[0][0] ** 2 + R[1][0] ** 2)
        singular = sy < 1e-6
        
        if not singular:
            rx = math.atan2(-R[2][1], R[2][2])
            ry = math.atan2(sy, -R[2][0])
            rz = math.atan2(R[1][0], R[0][0])
        else:
            rx = math.atan2(-R[2][1], R[2][2])
            ry = math.atan2(-sy, -R[2][0])
            rz = 0.0
        
        return {
            'x': x,
            'y': y,
            'z': z,
            'rx': math.degrees(rx),
            'ry': math.degrees(ry),
            'rz': math.degrees(rz),
        }
    
    def shutdown(self):
        """Выключение моста."""
        self.is_connected = False
        self.destroy_node()
