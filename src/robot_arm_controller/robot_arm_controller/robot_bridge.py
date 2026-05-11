import math
import threading
from typing import Callable, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class RobotBridge(Node):
    """
    Мост между PyQt GUI и ROS 2/Gazebo.

    Главная идея:
    - управление рукой идёт через /arm_controller/joint_trajectory;
    - управление захватом идёт через /gripper_controller/joint_trajectory;
    - прямая и обратная кинематика считаются не по старым UR5 DH-параметрам,
      а по реальным параметрам твоей URDF-цепочки из robot_arm.xacro.

    Это устраняет главную ошибку старой версии:
    раньше математическая модель и Gazebo-модель были разными.
    """

    ARM_JOINT_NAMES = [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    ]

    GRIPPER_JOINT_NAMES = [
        "left_finger_joint",
        "right_finger_joint",
    ]

    # Захват:
    # 0.000 м — закрыто
    # 0.025 м — открыто на 25 мм
    GRIPPER_CLOSED_POSITION = 0.0
    GRIPPER_OPEN_POSITION = 0.025

    # Реальная кинематическая цепочка из robot_arm.xacro.
    # Каждый joint задаётся так же, как в URDF:
    # parent -> origin xyz/rpy -> вращение вокруг локальной axis.
    #
    # Это не старые UR5 DH-параметры.
    # Это точная таблица твоей модели, по которой Gazebo строит робота.
    KINEMATIC_CHAIN = [
        {
            "name": "joint_1",
            "xyz": [0.0, 0.0, 0.1038],
            "rpy": [0.0, 0.0, math.pi],
            "axis": [0.0, 0.0, 1.0],
            "limit": [-3.14, 3.14],
        },
        {
            "name": "joint_2",
            "xyz": [-0.04375, -0.0558, 0.08],
            "rpy": [math.pi / 2.0, 0.0, 0.0],
            "axis": [0.0, 0.0, 1.0],
            "limit": [-0.7854, 1.5708],
        },
        {
            "name": "joint_3",
            "xyz": [0.0, 0.22, -0.04],
            "rpy": [0.0, math.pi, 0.0],
            "axis": [0.0, 0.0, 1.0],
            "limit": [-1.5708, 1.5708],
        },
        {
            "name": "joint_4",
            "xyz": [0.07, 0.075, 0.06],
            "rpy": [0.0, math.pi / 2.0, 0.0],
            "axis": [0.0, 0.0, 1.0],
            "limit": [-math.pi, math.pi],
        },
        {
            "name": "joint_5",
            "xyz": [0.0, 0.0, 0.172],
            "rpy": [math.pi / 2.0, 0.0, -math.pi / 2.0],
            "axis": [0.0, 0.0, 1.0],
            "limit": [-2.0944, 2.0944],
        },
        {
            "name": "joint_6",
            "xyz": [0.0, 0.08593, 0.0],
            "rpy": [-math.pi / 2.0, 0.0, 0.0],
            "axis": [0.0, 0.0, 1.0],
            "limit": [-3.14, 3.14],
        },
    ]

    TOOL_XYZ = [0.0, 0.0, 0.0867]
    TOOL_RPY = [0.0, 0.0, 0.0]

    def __init__(self):
        super().__init__("robot_arm_controller_v4")

        self.current_position: List[float] = [0.0] * 6
        self.current_gripper_position: List[float] = [0.0, 0.0]

        self.is_connected: bool = False
        self.callbacks: List[Callable[[Dict], None]] = []
        self.lock = threading.Lock()

        self.speed_scale = 1.0

        # Параметры IK.
        # Чем больше damping, тем устойчивее около сингулярностей,
        # но тем менее точно и медленнее сходится.
        self.ik_damping_pos = 0.035
        self.ik_damping_6d = 0.055
        self.ik_max_iterations = 70
        self.ik_position_tolerance = 0.0007
        self.ik_rotation_tolerance = math.radians(1.0)
        self.ik_max_joint_step = 0.045

        self.reference_orientation: Optional[List[List[float]]] = None

        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10,
        )

        self.gripper_publisher = self.create_publisher(
            JointTrajectory,
            "/gripper_controller/joint_trajectory",
            10,
        )

        self.state_subscriber = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.ros_thread = threading.Thread(target=self.spin_thread, daemon=True)
        self.ros_thread.start()

        self.is_connected = True

        self.get_logger().info("RobotBridge started with URDF-based kinematics.")

    # ------------------------------------------------------------------
    # ROS
    # ------------------------------------------------------------------

    def spin_thread(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def timer_callback(self):
        pass

    def joint_state_callback(self, msg: JointState):
        arm_positions: List[float] = []

        with self.lock:
            old_arm = self.current_position.copy()
            old_gripper = self.current_gripper_position.copy()

        for i, joint_name in enumerate(self.ARM_JOINT_NAMES):
            try:
                idx = msg.name.index(joint_name)
                arm_positions.append(float(msg.position[idx]))
            except ValueError:
                arm_positions.append(old_arm[i])

        gripper_positions: List[float] = []

        for i, joint_name in enumerate(self.GRIPPER_JOINT_NAMES):
            try:
                idx = msg.name.index(joint_name)
                gripper_positions.append(float(msg.position[idx]))
            except ValueError:
                gripper_positions.append(old_gripper[i])

        with self.lock:
            self.current_position = arm_positions
            self.current_gripper_position = gripper_positions

        pose = self.get_tcp_pose()
        self.notify_callbacks(
            {
                "position": arm_positions,
                "gripper": gripper_positions,
                "pose": pose,
            }
        )

    # ------------------------------------------------------------------
    # Базовая математика
    # ------------------------------------------------------------------

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _norm3(v: List[float]) -> float:
        return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

    @staticmethod
    def _dot3(a: List[float], b: List[float]) -> float:
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    @staticmethod
    def _cross3(a: List[float], b: List[float]) -> List[float]:
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]

    @staticmethod
    def _normalize3(v: List[float]) -> List[float]:
        n = RobotBridge._norm3(v)
        if n < 1e-12:
            return [0.0, 0.0, 1.0]
        return [v[0] / n, v[1] / n, v[2] / n]

    @staticmethod
    def _identity4() -> List[List[float]]:
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _matmul4(A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
        return [
            [
                A[i][0] * B[0][j]
                + A[i][1] * B[1][j]
                + A[i][2] * B[2][j]
                + A[i][3] * B[3][j]
                for j in range(4)
            ]
            for i in range(4)
        ]

    @staticmethod
    def _matmul3(A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
        return [
            [
                A[i][0] * B[0][j]
                + A[i][1] * B[1][j]
                + A[i][2] * B[2][j]
                for j in range(3)
            ]
            for i in range(3)
        ]

    @staticmethod
    def _matvec3(A: List[List[float]], v: List[float]) -> List[float]:
        return [
            A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
            A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
            A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
        ]

    @staticmethod
    def _transpose3(A: List[List[float]]) -> List[List[float]]:
        return [
            [A[0][0], A[1][0], A[2][0]],
            [A[0][1], A[1][1], A[2][1]],
            [A[0][2], A[1][2], A[2][2]],
        ]

    @staticmethod
    def _extract_R(T: List[List[float]]) -> List[List[float]]:
        return [
            [T[0][0], T[0][1], T[0][2]],
            [T[1][0], T[1][1], T[1][2]],
            [T[2][0], T[2][1], T[2][2]],
        ]

    @staticmethod
    def _extract_p(T: List[List[float]]) -> List[float]:
        return [T[0][3], T[1][3], T[2][3]]

    @staticmethod
    def _transform_from_R_p(
        R: List[List[float]],
        p: List[float],
    ) -> List[List[float]]:
        return [
            [R[0][0], R[0][1], R[0][2], p[0]],
            [R[1][0], R[1][1], R[1][2], p[1]],
            [R[2][0], R[2][1], R[2][2], p[2]],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _rot_x(angle: float) -> List[List[float]]:
        c = math.cos(angle)
        s = math.sin(angle)
        return [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ]

    @staticmethod
    def _rot_y(angle: float) -> List[List[float]]:
        c = math.cos(angle)
        s = math.sin(angle)
        return [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ]

    @staticmethod
    def _rot_z(angle: float) -> List[List[float]]:
        c = math.cos(angle)
        s = math.sin(angle)
        return [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _rpy_to_R(roll: float, pitch: float, yaw: float) -> List[List[float]]:
        # URDF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        Rz = RobotBridge._rot_z(yaw)
        Ry = RobotBridge._rot_y(pitch)
        Rx = RobotBridge._rot_x(roll)
        return RobotBridge._matmul3(RobotBridge._matmul3(Rz, Ry), Rx)

    @staticmethod
    def _origin_transform(
        xyz: List[float],
        rpy: List[float],
    ) -> List[List[float]]:
        R = RobotBridge._rpy_to_R(rpy[0], rpy[1], rpy[2])
        return RobotBridge._transform_from_R_p(R, xyz)

    @staticmethod
    def _axis_angle_R(axis: List[float], angle: float) -> List[List[float]]:
        ax = RobotBridge._normalize3(axis)
        x, y, z = ax

        c = math.cos(angle)
        s = math.sin(angle)
        v = 1.0 - c

        return [
            [c + x * x * v, x * y * v - z * s, x * z * v + y * s],
            [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
            [z * x * v - y * s, z * y * v + x * s, c + z * z * v],
        ]

    @staticmethod
    def _axis_angle_transform(
        axis: List[float],
        angle: float,
    ) -> List[List[float]]:
        R = RobotBridge._axis_angle_R(axis, angle)
        return RobotBridge._transform_from_R_p(R, [0.0, 0.0, 0.0])

    @staticmethod
    def _so3_log(R: List[List[float]]) -> List[float]:
        trace = R[0][0] + R[1][1] + R[2][2]
        cos_theta = RobotBridge._clamp((trace - 1.0) / 2.0, -1.0, 1.0)
        theta = math.acos(cos_theta)

        if theta < 1e-8:
            return [0.0, 0.0, 0.0]

        if abs(math.pi - theta) < 1e-4:
            # Защита около 180 градусов.
            xx = max(0.0, (R[0][0] + 1.0) / 2.0)
            yy = max(0.0, (R[1][1] + 1.0) / 2.0)
            zz = max(0.0, (R[2][2] + 1.0) / 2.0)

            x = math.sqrt(xx)
            y = math.sqrt(yy)
            z = math.sqrt(zz)

            if R[0][1] < 0:
                y = -y
            if R[0][2] < 0:
                z = -z

            axis = RobotBridge._normalize3([x, y, z])
            return [theta * axis[0], theta * axis[1], theta * axis[2]]

        s = math.sin(theta)

        return [
            theta * (R[2][1] - R[1][2]) / (2.0 * s),
            theta * (R[0][2] - R[2][0]) / (2.0 * s),
            theta * (R[1][0] - R[0][1]) / (2.0 * s),
        ]

    # ------------------------------------------------------------------
    # Кватернионы
    # ------------------------------------------------------------------

    @staticmethod
    def quaternion_normalize(q: List[float]) -> List[float]:
        x, y, z, w = q
        n = math.sqrt(x * x + y * y + z * z + w * w)

        if n < 1e-12:
            return [0.0, 0.0, 0.0, 1.0]

        return [x / n, y / n, z / n, w / n]

    @staticmethod
    def quaternion_from_rotation_matrix(R: List[List[float]]) -> List[float]:
        trace = R[0][0] + R[1][1] + R[2][2]

        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (R[2][1] - R[1][2]) / s
            y = (R[0][2] - R[2][0]) / s
            z = (R[1][0] - R[0][1]) / s
        elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
            s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
            w = (R[2][1] - R[1][2]) / s
            x = 0.25 * s
            y = (R[0][1] + R[1][0]) / s
            z = (R[0][2] + R[2][0]) / s
        elif R[1][1] > R[2][2]:
            s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
            w = (R[0][2] - R[2][0]) / s
            x = (R[0][1] + R[1][0]) / s
            y = 0.25 * s
            z = (R[1][2] + R[2][1]) / s
        else:
            s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
            w = (R[1][0] - R[0][1]) / s
            x = (R[0][2] + R[2][0]) / s
            y = (R[1][2] + R[2][1]) / s
            z = 0.25 * s

        return RobotBridge.quaternion_normalize([x, y, z, w])

    # ------------------------------------------------------------------
    # Прямая кинематика по реальной URDF-цепочке
    # ------------------------------------------------------------------

    def clamp_to_joint_limits(self, q: List[float]) -> List[float]:
        result = []

        for i, value in enumerate(q):
            low, high = self.KINEMATIC_CHAIN[i]["limit"]
            result.append(self._clamp(value, low, high))

        return result

    def fk_transform(
        self,
        joints: List[float],
        include_tool: bool = True,
    ) -> List[List[float]]:
        q = self.clamp_to_joint_limits(joints)
        T = self._identity4()

        for i, joint in enumerate(self.KINEMATIC_CHAIN):
            T_origin = self._origin_transform(joint["xyz"], joint["rpy"])
            T_joint = self._axis_angle_transform(joint["axis"], q[i])

            T = self._matmul4(T, T_origin)
            T = self._matmul4(T, T_joint)

        if include_tool:
            T_tool = self._origin_transform(self.TOOL_XYZ, self.TOOL_RPY)
            T = self._matmul4(T, T_tool)

        return T

    def forward_kinematics(self, joints: List[float]) -> Tuple[float, float, float]:
        T = self.fk_transform(joints, include_tool=True)
        p = self._extract_p(T)
        return p[0], p[1], p[2]

    def forward_kinematics_full(
        self,
        joints: List[float],
    ) -> Tuple[List[List[float]], List[float]]:
        T = self.fk_transform(joints, include_tool=True)
        R = self._extract_R(T)
        p = self._extract_p(T)
        return R, p

    def compute_geometric_jacobian(
        self,
        joints: List[float],
    ) -> Tuple[List[List[float]], List[List[float]]]:
        """
        Геометрический якобиан 6-осевого манипулятора.

        Возвращает:
        - J_pos: 3x6, линейная часть
        - J_rot: 3x6, угловая часть

        Для revolute joint:
        Jv_i = z_i x (p_tcp - p_i)
        Jw_i = z_i
        """

        q = self.clamp_to_joint_limits(joints)
        T = self._identity4()

        joint_origins: List[List[float]] = []
        joint_axes_world: List[List[float]] = []

        for i, joint in enumerate(self.KINEMATIC_CHAIN):
            T_origin = self._origin_transform(joint["xyz"], joint["rpy"])
            T = self._matmul4(T, T_origin)

            R_origin_world = self._extract_R(T)
            p_origin_world = self._extract_p(T)

            axis_world = self._matvec3(R_origin_world, joint["axis"])
            axis_world = self._normalize3(axis_world)

            joint_origins.append(p_origin_world)
            joint_axes_world.append(axis_world)

            T_joint = self._axis_angle_transform(joint["axis"], q[i])
            T = self._matmul4(T, T_joint)

        T_tool = self._origin_transform(self.TOOL_XYZ, self.TOOL_RPY)
        T_tcp = self._matmul4(T, T_tool)
        p_tcp = self._extract_p(T_tcp)

        J_pos_columns: List[List[float]] = []
        J_rot_columns: List[List[float]] = []

        for i in range(6):
            axis = joint_axes_world[i]
            origin = joint_origins[i]
            r = [
                p_tcp[0] - origin[0],
                p_tcp[1] - origin[1],
                p_tcp[2] - origin[2],
            ]

            linear = self._cross3(axis, r)
            angular = axis

            J_pos_columns.append(linear)
            J_rot_columns.append(angular)

        J_pos = [
            [J_pos_columns[j][0] for j in range(6)],
            [J_pos_columns[j][1] for j in range(6)],
            [J_pos_columns[j][2] for j in range(6)],
        ]

        J_rot = [
            [J_rot_columns[j][0] for j in range(6)],
            [J_rot_columns[j][1] for j in range(6)],
            [J_rot_columns[j][2] for j in range(6)],
        ]

        return J_pos, J_rot

    # Старые имена оставлены, чтобы не ломать остальной код.
    def compute_position_jacobian(self, joints: List[float]) -> List[List[float]]:
        J_pos, _ = self.compute_geometric_jacobian(joints)
        return J_pos

    def compute_orientation_jacobian(self, joints: List[float]) -> List[List[float]]:
        _, J_rot = self.compute_geometric_jacobian(joints)
        return J_rot

    # ------------------------------------------------------------------
    # Damped Least Squares IK
    # ------------------------------------------------------------------

    def _solve_linear_system(
        self,
        A: List[List[float]],
        b: List[float],
    ) -> List[float]:
        n = len(b)

        M = [row[:] for row in A]
        x = b[:]

        for i in range(n):
            pivot_row = i
            pivot_abs = abs(M[i][i])

            for r in range(i + 1, n):
                candidate = abs(M[r][i])
                if candidate > pivot_abs:
                    pivot_abs = candidate
                    pivot_row = r

            if pivot_row != i:
                M[i], M[pivot_row] = M[pivot_row], M[i]
                x[i], x[pivot_row] = x[pivot_row], x[i]

            pivot = M[i][i]

            if abs(pivot) < 1e-12:
                M[i][i] = 1e-12
                pivot = M[i][i]

            inv_pivot = 1.0 / pivot

            for j in range(i, n):
                M[i][j] *= inv_pivot

            x[i] *= inv_pivot

            for r in range(n):
                if r == i:
                    continue

                factor = M[r][i]

                if abs(factor) < 1e-15:
                    continue

                for j in range(i, n):
                    M[r][j] -= factor * M[i][j]

                x[r] -= factor * x[i]

        return x

    def _damped_least_squares(
        self,
        J: List[List[float]],
        error: List[float],
        damping: float,
    ) -> List[float]:
        """
        dq = J^T (J J^T + lambda^2 I)^-1 error
        """

        rows = len(J)
        cols = len(J[0])

        A = [[0.0 for _ in range(rows)] for _ in range(rows)]

        for i in range(rows):
            for j in range(rows):
                A[i][j] = sum(J[i][k] * J[j][k] for k in range(cols))

        for i in range(rows):
            A[i][i] += damping * damping

        y = self._solve_linear_system(A, error)

        dq = [0.0 for _ in range(cols)]

        for j in range(cols):
            dq[j] = sum(J[i][j] * y[i] for i in range(rows))

        return dq

    def _limit_delta_q(self, dq: List[float], max_step: float) -> List[float]:
        max_abs = max(abs(v) for v in dq) if dq else 0.0

        if max_abs <= max_step or max_abs < 1e-12:
            return dq

        scale = max_step / max_abs
        return [v * scale for v in dq]

    def solve_ik(
        self,
        target_position: List[float],
        target_rotation: Optional[List[List[float]]] = None,
        initial_joints: Optional[List[float]] = None,
        duration_hint: float = 0.5,
        keep_orientation: bool = False,
    ) -> Tuple[List[float], bool, float, float]:
        """
        Численная IK по реальной URDF-цепочке.

        Если target_rotation is None:
            решается только позиция TCP.

        Если target_rotation задан:
            решается позиция + ориентация.

        Возвращает:
            q_solution, success, position_error, rotation_error
        """

        if initial_joints is None:
            with self.lock:
                q = self.current_position.copy()
        else:
            q = initial_joints.copy()

        q = self.clamp_to_joint_limits(q)

        use_orientation = target_rotation is not None or keep_orientation

        if keep_orientation and target_rotation is None:
            target_rotation, _ = self.forward_kinematics_full(q)

        last_pos_error = 999.0
        last_rot_error = 999.0

        for _ in range(self.ik_max_iterations):
            R_current, p_current = self.forward_kinematics_full(q)

            e_pos = [
                target_position[0] - p_current[0],
                target_position[1] - p_current[1],
                target_position[2] - p_current[2],
            ]

            pos_error = self._norm3(e_pos)
            last_pos_error = pos_error

            J_pos, J_rot = self.compute_geometric_jacobian(q)

            if use_orientation and target_rotation is not None:
                # Ошибка ориентации в базовой системе координат:
                # R_err = R_target * R_current^T
                R_err = self._matmul3(target_rotation, self._transpose3(R_current))
                e_rot = self._so3_log(R_err)

                rot_error = self._norm3(e_rot)
                last_rot_error = rot_error

                if (
                    pos_error < self.ik_position_tolerance
                    and rot_error < self.ik_rotation_tolerance
                ):
                    return q, True, pos_error, rot_error

                # Для учебного манипулятора ориентация не должна убивать
                # линейное движение. Поэтому вес ориентации умеренный.
                rot_weight = 0.35

                J = [
                    J_pos[0],
                    J_pos[1],
                    J_pos[2],
                    [rot_weight * value for value in J_rot[0]],
                    [rot_weight * value for value in J_rot[1]],
                    [rot_weight * value for value in J_rot[2]],
                ]

                error = [
                    e_pos[0],
                    e_pos[1],
                    e_pos[2],
                    rot_weight * e_rot[0],
                    rot_weight * e_rot[1],
                    rot_weight * e_rot[2],
                ]

                dq = self._damped_least_squares(J, error, self.ik_damping_6d)

            else:
                last_rot_error = 0.0

                if pos_error < self.ik_position_tolerance:
                    return q, True, pos_error, 0.0

                dq = self._damped_least_squares(J_pos, e_pos, self.ik_damping_pos)

            dq = self._limit_delta_q(dq, self.ik_max_joint_step)

            q = [q[i] + dq[i] for i in range(6)]
            q = self.clamp_to_joint_limits(q)

        return q, False, last_pos_error, last_rot_error

    # Старые методы оставлены для совместимости.
    def ik_step(
        self,
        joints: List[float],
        dx: float,
        dy: float,
        dz: float,
        step_scale: float = 0.7,
    ) -> List[float]:
        _, p_current = self.forward_kinematics_full(joints)
        target = [
            p_current[0] + dx,
            p_current[1] + dy,
            p_current[2] + dz,
        ]

        q, _, _, _ = self.solve_ik(
            target_position=target,
            target_rotation=None,
            initial_joints=joints,
            duration_hint=0.2,
        )

        return q

    def ik_step_6d(
        self,
        joints: List[float],
        Rd: List[List[float]],
        pd: List[float],
        step_scale_pos: float = 1.0,
        step_scale_rot: float = 1.0,
        lam: float = 1e-2,
    ) -> List[float]:
        q, _, _, _ = self.solve_ik(
            target_position=pd,
            target_rotation=Rd,
            initial_joints=joints,
            duration_hint=0.2,
        )

        return q

    # ------------------------------------------------------------------
    # Управление рукой
    # ------------------------------------------------------------------

    @staticmethod
    def _duration_from_float(duration_sec: float) -> Duration:
        duration_sec = max(0.0, float(duration_sec))
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        return Duration(sec=sec, nanosec=nanosec)

    def send_trajectory(
        self,
        joint_angles: List[float],
        duration_sec: float = 1.0,
    ):
        if len(joint_angles) != 6:
            self.get_logger().error(
                f"send_trajectory expected 6 joint angles, got {len(joint_angles)}"
            )
            return

        q = self.clamp_to_joint_limits([float(v) for v in joint_angles])

        traj = JointTrajectory()
        traj.joint_names = self.ARM_JOINT_NAMES.copy()

        point = JointTrajectoryPoint()
        point.positions = q
        point.time_from_start = self._duration_from_float(duration_sec)

        traj.points.append(point)
        self.trajectory_publisher.publish(traj)

    def move_end_effector_world(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        duration: float = 0.8,
    ) -> bool:
        """
        Линейное смещение TCP в мировой системе координат.

        Движение рассчитывается через IK по реальной URDF-кинематике.
        Ориентация TCP сохраняется.
        """

        try:
            if abs(dx) < 1e-12 and abs(dy) < 1e-12 and abs(dz) < 1e-12:
                return True

            # speed_scale уже учитывается в manual_control_ui при расчёте шага.
            # Дополнительно масштабировать dx/dy/dz здесь нельзя, иначе скорость
            # будет умножаться дважды.
            with self.lock:
                q_start = self.current_position.copy()

            R_start, p_start = self.forward_kinematics_full(q_start)

            target_position = [
                p_start[0] + dx,
                p_start[1] + dy,
                p_start[2] + dz,
            ]

            q_solution, success, pos_error, rot_error = self.solve_ik(
                target_position=target_position,
                target_rotation=R_start,
                initial_joints=q_start,
                duration_hint=duration,
                keep_orientation=True,
            )

            self.send_trajectory(q_solution, duration)

            if not success:
                self.get_logger().warn(
                    "IK linear move not fully converged: "
                    f"pos_error={pos_error:.6f}, rot_error={rot_error:.6f}"
                )

            return success

        except Exception as exc:
            self.get_logger().error(f"move_end_effector_world error: {exc}")
            return False

    def rotate_end_effector_rx_ry_ik(
        self,
        d_rx: float = 0.0,
        d_ry: float = 0.0,
        duration: float = 0.1,
    ) -> bool:
        """
        Поворот TCP вокруг его локальных RX/RY с сохранением позиции TCP.
        """

        try:
            if abs(d_rx) < 1e-12 and abs(d_ry) < 1e-12:
                return True

            with self.lock:
                q_start = self.current_position.copy()

            R_start, p_start = self.forward_kinematics_full(q_start)

            R_delta_x = self._rot_x(d_rx)
            R_delta_y = self._rot_y(d_ry)
            R_delta = self._matmul3(R_delta_x, R_delta_y)

            # Локальный поворот TCP:
            # R_target = R_current * R_delta
            R_target = self._matmul3(R_start, R_delta)

            q_solution, success, pos_error, rot_error = self.solve_ik(
                target_position=p_start,
                target_rotation=R_target,
                initial_joints=q_start,
                duration_hint=duration,
                keep_orientation=False,
            )

            self.send_trajectory(q_solution, duration)

            if not success:
                self.get_logger().warn(
                    "IK RX/RY rotation not fully converged: "
                    f"pos_error={pos_error:.6f}, rot_error={rot_error:.6f}"
                )

            return success

        except Exception as exc:
            self.get_logger().error(f"rotate_end_effector_rx_ry_ik error: {exc}")
            return False

    def rotate_end_effector_world(
        self,
        drz: float = 0.0,
        duration: float = 0.1,
    ) -> bool:
        """
        Поворот TCP вокруг локальной оси Z.

        Раньше здесь просто менялся joint_6.
        Теперь движение считается через IK, чтобы TCP оставался на месте.
        """

        try:
            if abs(drz) < 1e-12:
                return True

            with self.lock:
                q_start = self.current_position.copy()

            R_start, p_start = self.forward_kinematics_full(q_start)

            R_delta = self._rot_z(drz)
            R_target = self._matmul3(R_start, R_delta)

            q_solution, success, pos_error, rot_error = self.solve_ik(
                target_position=p_start,
                target_rotation=R_target,
                initial_joints=q_start,
                duration_hint=duration,
                keep_orientation=False,
            )

            self.send_trajectory(q_solution, duration)

            if not success:
                self.get_logger().warn(
                    "IK RZ rotation not fully converged: "
                    f"pos_error={pos_error:.6f}, rot_error={rot_error:.6f}"
                )

            return success

        except Exception as exc:
            self.get_logger().error(f"rotate_end_effector_world error: {exc}")
            return False

    def move_joint(
        self,
        joint_index: int,
        angle: float,
        duration_sec: float = 0.5,
    ):
        """
        Управление отдельным суставом.

        Поддерживаются оба варианта:
        - joint_index = 0..5;
        - joint_index = 1..6.

        Для DSL удобнее писать joint_set(1, 30, 2),
        поэтому значение 1 считается первым суставом.
        """

        if 1 <= joint_index <= 6:
            idx = joint_index - 1
        elif 0 <= joint_index <= 5:
            idx = joint_index
        else:
            self.get_logger().error(f"Invalid joint index: {joint_index}")
            return

        with self.lock:
            q = self.current_position.copy()

        q[idx] = float(angle)
        q = self.clamp_to_joint_limits(q)

        self.send_trajectory(q, duration_sec)

    def move_joints_absolute(
        self,
        angles: List[float],
        duration_sec: float = 1.0,
    ):
        if len(angles) != 6:
            self.get_logger().error(
                f"move_joints_absolute expected 6 angles, got {len(angles)}"
            )
            return

        self.send_trajectory(angles, duration_sec)

    def reset_position(self):
        """
        Правильная исходная позиция, которую ты проверила визуально:
        все суставы = 0 градусов.
        """

        home = [
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0),
        ]

        self.send_trajectory(home, 2.0)

    # ------------------------------------------------------------------
    # Управление захватом
    # ------------------------------------------------------------------

    def send_gripper_trajectory(
        self,
        opening: float,
        duration_sec: float = 0.7,
    ):
        """
        Управление захватом.

        opening:
        - 0.000 — закрыто
        - 0.025 — открыто на 25 мм
        """

        opening = self._clamp(
            float(opening),
            self.GRIPPER_CLOSED_POSITION,
            self.GRIPPER_OPEN_POSITION,
        )

        traj = JointTrajectory()
        traj.joint_names = self.GRIPPER_JOINT_NAMES.copy()

        point = JointTrajectoryPoint()
        point.positions = [opening, opening]
        point.time_from_start = self._duration_from_float(duration_sec)

        traj.points.append(point)
        self.gripper_publisher.publish(traj)

    def open_gripper(self, duration_sec: float = 0.7):
        self.send_gripper_trajectory(self.GRIPPER_OPEN_POSITION, duration_sec)

    def close_gripper(self, duration_sec: float = 0.7):
        self.send_gripper_trajectory(self.GRIPPER_CLOSED_POSITION, duration_sec)

    # ------------------------------------------------------------------
    # Ориентация-референс для program_editor_ui.py
    # ------------------------------------------------------------------

    def save_reference_orientation(self):
        R, _ = self.forward_kinematics_full(self.get_current_position())
        self.reference_orientation = R
        self.get_logger().info("Reference TCP orientation saved.")

    def align_orientation_to_reference(self, duration_sec: float = 0.3):
        if self.reference_orientation is None:
            self.get_logger().warn("Reference orientation is not saved yet.")
            return False

        with self.lock:
            q_start = self.current_position.copy()

        _, p_current = self.forward_kinematics_full(q_start)

        q_solution, success, pos_error, rot_error = self.solve_ik(
            target_position=p_current,
            target_rotation=self.reference_orientation,
            initial_joints=q_start,
            duration_hint=duration_sec,
            keep_orientation=False,
        )

        self.send_trajectory(q_solution, duration_sec)

        if not success:
            self.get_logger().warn(
                "align_orientation_to_reference not fully converged: "
                f"pos_error={pos_error:.6f}, rot_error={rot_error:.6f}"
            )

        return success

    # ------------------------------------------------------------------
    # Получение состояния
    # ------------------------------------------------------------------

    def get_current_position(self) -> List[float]:
        with self.lock:
            return self.current_position.copy()

    def get_current_gripper_position(self) -> List[float]:
        with self.lock:
            return self.current_gripper_position.copy()

    def get_tcp_position(self) -> List[float]:
        x, y, z = self.forward_kinematics(self.get_current_position())
        return [x, y, z]

    def get_tcp_orientation(self) -> List[float]:
        R, _ = self.forward_kinematics_full(self.get_current_position())
        return self.quaternion_from_rotation_matrix(R)

    def get_tcp_pose(self) -> Dict[str, float]:
        q = self.get_current_position()
        R, p = self.forward_kinematics_full(q)

        # Euler XYZ-like display for GUI only.
        sy = math.sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0])
        singular = sy < 1e-6

        if not singular:
            rx = math.atan2(R[2][1], R[2][2])
            ry = math.atan2(-R[2][0], sy)
            rz = math.atan2(R[1][0], R[0][0])
        else:
            rx = math.atan2(-R[1][2], R[1][1])
            ry = math.atan2(-R[2][0], sy)
            rz = 0.0

        # Сохраняю старую логику отображения X/Y,
        # чтобы интерфейс не поменял привычные знаки координат.
        return {
            "x": -p[0],
            "y": -p[1],
            "z": p[2],
            "rx": math.degrees(rx),
            "ry": math.degrees(ry),
            "rz": math.degrees(rz),
        }

    # ------------------------------------------------------------------
    # Callback-система для GUI
    # ------------------------------------------------------------------

    def register_callback(self, callback: Callable[[Dict], None]):
        self.callbacks.append(callback)

    def notify_callbacks(self, data: Dict):
        for callback in self.callbacks:
            try:
                callback(data)
            except Exception as exc:
                self.get_logger().warn(f"GUI callback error: {exc}")

    def shutdown(self):
        self.is_connected = False
        self.destroy_node()