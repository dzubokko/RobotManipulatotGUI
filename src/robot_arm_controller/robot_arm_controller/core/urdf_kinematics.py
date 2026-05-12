import math
from typing import List, Optional, Tuple

from robot_arm_controller.core.robot_model import RobotModel


class UrdfKinematics:
    """
    Кинематика робота по эталонной URDF-цепочке.

    Это core-версия старой рабочей логики:
    - FK считается по xyz/rpy/axis;
    - IK решается через Damped Least Squares;
    - параметры цепочки вынесены в RobotModel.
    """

    def __init__(self, model: RobotModel):
        self.model = model

        self.ik_damping_pos = 0.035
        self.ik_damping_6d = 0.055
        self.ik_max_iterations = 70
        self.ik_position_tolerance = 0.0007
        self.ik_rotation_tolerance = math.radians(1.0)
        self.ik_max_joint_step = 0.045

    # ------------------------------------------------------------------
    # Basic math
    # ------------------------------------------------------------------

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def norm3(v: List[float]) -> float:
        return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

    @staticmethod
    def cross3(a: List[float], b: List[float]) -> List[float]:
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]

    @staticmethod
    def normalize3(v: List[float]) -> List[float]:
        n = UrdfKinematics.norm3(v)

        if n < 1e-12:
            return [0.0, 0.0, 1.0]

        return [v[0] / n, v[1] / n, v[2] / n]

    @staticmethod
    def identity4() -> List[List[float]]:
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def matmul4(A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
        result = [[0.0 for _ in range(4)] for _ in range(4)]

        for i in range(4):
            for j in range(4):
                result[i][j] = (
                    A[i][0] * B[0][j]
                    + A[i][1] * B[1][j]
                    + A[i][2] * B[2][j]
                    + A[i][3] * B[3][j]
                )

        return result

    @staticmethod
    def matmul3(A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
        result = [[0.0 for _ in range(3)] for _ in range(3)]

        for i in range(3):
            for j in range(3):
                result[i][j] = (
                    A[i][0] * B[0][j]
                    + A[i][1] * B[1][j]
                    + A[i][2] * B[2][j]
                )

        return result

    @staticmethod
    def matvec3(A: List[List[float]], v: List[float]) -> List[float]:
        return [
            A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
            A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
            A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
        ]

    @staticmethod
    def transpose3(A: List[List[float]]) -> List[List[float]]:
        return [
            [A[0][0], A[1][0], A[2][0]],
            [A[0][1], A[1][1], A[2][1]],
            [A[0][2], A[1][2], A[2][2]],
        ]

    @staticmethod
    def extract_R(T: List[List[float]]) -> List[List[float]]:
        return [
            [T[0][0], T[0][1], T[0][2]],
            [T[1][0], T[1][1], T[1][2]],
            [T[2][0], T[2][1], T[2][2]],
        ]

    @staticmethod
    def extract_p(T: List[List[float]]) -> List[float]:
        return [T[0][3], T[1][3], T[2][3]]

    @staticmethod
    def transform_from_R_p(
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
    def rot_x(angle: float) -> List[List[float]]:
        c = math.cos(angle)
        s = math.sin(angle)

        return [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ]

    @staticmethod
    def rot_y(angle: float) -> List[List[float]]:
        c = math.cos(angle)
        s = math.sin(angle)

        return [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ]

    @staticmethod
    def rot_z(angle: float) -> List[List[float]]:
        c = math.cos(angle)
        s = math.sin(angle)

        return [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ]

    @staticmethod
    def rpy_to_R(roll: float, pitch: float, yaw: float) -> List[List[float]]:
        return UrdfKinematics.matmul3(
            UrdfKinematics.matmul3(
                UrdfKinematics.rot_z(yaw),
                UrdfKinematics.rot_y(pitch),
            ),
            UrdfKinematics.rot_x(roll),
        )

    @staticmethod
    def origin_transform(
        xyz: List[float],
        rpy: List[float],
    ) -> List[List[float]]:
        R = UrdfKinematics.rpy_to_R(rpy[0], rpy[1], rpy[2])
        return UrdfKinematics.transform_from_R_p(R, xyz)

    @staticmethod
    def axis_angle_R(axis: List[float], angle: float) -> List[List[float]]:
        x, y, z = UrdfKinematics.normalize3(axis)

        c = math.cos(angle)
        s = math.sin(angle)
        v = 1.0 - c

        return [
            [c + x * x * v, x * y * v - z * s, x * z * v + y * s],
            [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
            [z * x * v - y * s, z * y * v + x * s, c + z * z * v],
        ]

    @staticmethod
    def axis_angle_transform(
        axis: List[float],
        angle: float,
    ) -> List[List[float]]:
        R = UrdfKinematics.axis_angle_R(axis, angle)
        return UrdfKinematics.transform_from_R_p(R, [0.0, 0.0, 0.0])

    @staticmethod
    def so3_log(R: List[List[float]]) -> List[float]:
        trace = R[0][0] + R[1][1] + R[2][2]
        cos_theta = UrdfKinematics.clamp((trace - 1.0) / 2.0, -1.0, 1.0)
        theta = math.acos(cos_theta)

        if theta < 1e-8:
            return [0.0, 0.0, 0.0]

        if abs(math.pi - theta) < 1e-4:
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

            axis = UrdfKinematics.normalize3([x, y, z])

            return [
                theta * axis[0],
                theta * axis[1],
                theta * axis[2],
            ]

        s = math.sin(theta)

        return [
            theta * (R[2][1] - R[1][2]) / (2.0 * s),
            theta * (R[0][2] - R[2][0]) / (2.0 * s),
            theta * (R[1][0] - R[0][1]) / (2.0 * s),
        ]

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

        return UrdfKinematics.quaternion_normalize([x, y, z, w])

    # ------------------------------------------------------------------
    # FK / Jacobian
    # ------------------------------------------------------------------

    def clamp_to_joint_limits(self, q: List[float]) -> List[float]:
        result = []

        for i, value in enumerate(q):
            low, high = self.model.kinematic_chain[i].limit
            result.append(self.clamp(float(value), low, high))

        return result

    def fk_transform(
        self,
        joints: List[float],
        include_tool: bool = True,
    ) -> List[List[float]]:
        q = self.clamp_to_joint_limits(joints)
        T = self.identity4()

        for i, joint in enumerate(self.model.kinematic_chain):
            T_origin = self.origin_transform(joint.xyz, joint.rpy)
            T_joint = self.axis_angle_transform(joint.axis, q[i])

            T = self.matmul4(T, T_origin)
            T = self.matmul4(T, T_joint)

        if include_tool:
            T_tool = self.origin_transform(self.model.tool_xyz, self.model.tool_rpy)
            T = self.matmul4(T, T_tool)

        return T

    def forward_kinematics(self, joints: List[float]) -> Tuple[float, float, float]:
        T = self.fk_transform(joints, include_tool=True)
        p = self.extract_p(T)

        return p[0], p[1], p[2]

    def forward_kinematics_full(
        self,
        joints: List[float],
    ) -> Tuple[List[List[float]], List[float]]:
        T = self.fk_transform(joints, include_tool=True)
        R = self.extract_R(T)
        p = self.extract_p(T)

        return R, p

    def compute_geometric_jacobian(
        self,
        joints: List[float],
    ) -> Tuple[List[List[float]], List[List[float]]]:
        q = self.clamp_to_joint_limits(joints)
        T = self.identity4()

        joint_origins: List[List[float]] = []
        joint_axes_world: List[List[float]] = []

        for i, joint in enumerate(self.model.kinematic_chain):
            T_origin = self.origin_transform(joint.xyz, joint.rpy)
            T = self.matmul4(T, T_origin)

            R_origin_world = self.extract_R(T)
            p_origin_world = self.extract_p(T)

            axis_world = self.matvec3(R_origin_world, joint.axis)
            axis_world = self.normalize3(axis_world)

            joint_origins.append(p_origin_world)
            joint_axes_world.append(axis_world)

            T_joint = self.axis_angle_transform(joint.axis, q[i])
            T = self.matmul4(T, T_joint)

        T_tool = self.origin_transform(self.model.tool_xyz, self.model.tool_rpy)
        T_tcp = self.matmul4(T, T_tool)
        p_tcp = self.extract_p(T_tcp)

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

            linear = self.cross3(axis, r)
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

    def compute_position_jacobian(self, joints: List[float]) -> List[List[float]]:
        J_pos, _ = self.compute_geometric_jacobian(joints)
        return J_pos

    def compute_orientation_jacobian(self, joints: List[float]) -> List[List[float]]:
        _, J_rot = self.compute_geometric_jacobian(joints)
        return J_rot

    # ------------------------------------------------------------------
    # Damped Least Squares IK
    # ------------------------------------------------------------------

    def solve_linear_system(
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
                pivot = 1e-12
                M[i][i] = pivot

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

    def damped_least_squares(
        self,
        J: List[List[float]],
        error: List[float],
        damping: float,
    ) -> List[float]:
        rows = len(J)
        cols = len(J[0])

        A = [[0.0 for _ in range(rows)] for _ in range(rows)]

        for i in range(rows):
            for j in range(rows):
                A[i][j] = sum(J[i][k] * J[j][k] for k in range(cols))

        for i in range(rows):
            A[i][i] += damping * damping

        y = self.solve_linear_system(A, error)

        dq = [0.0 for _ in range(cols)]

        for j in range(cols):
            dq[j] = sum(J[i][j] * y[i] for i in range(rows))

        return dq

    def limit_delta_q(self, dq: List[float], max_step: float) -> List[float]:
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
        if initial_joints is None:
            q = [0.0] * 6
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

            pos_error = self.norm3(e_pos)
            last_pos_error = pos_error

            J_pos, J_rot = self.compute_geometric_jacobian(q)

            if use_orientation and target_rotation is not None:
                R_err = self.matmul3(target_rotation, self.transpose3(R_current))
                e_rot = self.so3_log(R_err)

                rot_error = self.norm3(e_rot)
                last_rot_error = rot_error

                if (
                    pos_error < self.ik_position_tolerance
                    and rot_error < self.ik_rotation_tolerance
                ):
                    return q, True, pos_error, rot_error

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

                dq = self.damped_least_squares(J, error, self.ik_damping_6d)

            else:
                last_rot_error = 0.0

                if pos_error < self.ik_position_tolerance:
                    return q, True, pos_error, 0.0

                dq = self.damped_least_squares(J_pos, e_pos, self.ik_damping_pos)

            dq = self.limit_delta_q(dq, self.ik_max_joint_step)

            q = [q[i] + dq[i] for i in range(6)]
            q = self.clamp_to_joint_limits(q)

        return q, False, last_pos_error, last_rot_error

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