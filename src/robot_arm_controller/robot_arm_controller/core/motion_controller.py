import math
from typing import Callable, List, Optional, Tuple

from robot_arm_controller.core.robot_model import RobotModel
from robot_arm_controller.core.urdf_kinematics import UrdfKinematics


class MotionController:
    """
    High-level arm motion logic.

    This class does not create ROS publishers or subscribers. It calculates
    joint targets and gives them to RobotBridge through callback functions.
    """

    def __init__(
        self,
        model: RobotModel,
        kinematics: UrdfKinematics,
        get_current_joints: Callable[[], List[float]],
        publish_arm_points: Callable[[List[List[float]], float], bool],
        logger=None,
    ) -> None:
        self.model = model
        self.kin = kinematics
        self.get_current_joints = get_current_joints
        self.publish_arm_points = publish_arm_points
        self.logger = logger
        self.reference_orientation: Optional[List[List[float]]] = None

    def _info(self, message: str) -> None:
        if self.logger is not None:
            self.logger.info(message)

    def _warn(self, message: str) -> None:
        if self.logger is not None:
            self.logger.warn(message)

    def _error(self, message: str) -> None:
        if self.logger is not None:
            self.logger.error(message)

    def send_joint_target(self, joints: List[float], duration_sec: float = 1.0) -> bool:
        if len(joints) != self.model.joint_count:
            self._error(
                f"send_joint_target expected {self.model.joint_count} joints, "
                f"got {len(joints)}"
            )
            return False

        q = self.kin.clamp_to_joint_limits([float(value) for value in joints])
        return self.publish_arm_points([q], duration_sec)

    def send_joint_trajectory(
        self,
        points: List[List[float]],
        duration_sec: float,
    ) -> bool:
        if not points:
            self._warn("send_joint_trajectory called with empty point list")
            return False

        clamped_points = []
        for point in points:
            if len(point) != self.model.joint_count:
                self._error(
                    f"Trajectory point expected {self.model.joint_count} joints, "
                    f"got {len(point)}"
                )
                return False
            clamped_points.append(self.kin.clamp_to_joint_limits(point))

        return self.publish_arm_points(clamped_points, duration_sec)

    def move_joint(
        self,
        joint_index: int,
        angle: float,
        duration_sec: float = 0.5,
    ) -> bool:
        """
        Move one joint.

        Supports both external numbering 1..6 and internal numbering 0..5.
        """
        if 1 <= joint_index <= self.model.joint_count:
            idx = joint_index - 1
        elif 0 <= joint_index < self.model.joint_count:
            idx = joint_index
        else:
            self._error(f"Invalid joint index: {joint_index}")
            return False

        q = self.get_current_joints()
        q[idx] = float(angle)
        q = self.kin.clamp_to_joint_limits(q)
        return self.send_joint_target(q, duration_sec)

    def move_joints_absolute(
        self,
        angles: List[float],
        duration_sec: float = 1.0,
    ) -> bool:
        return self.send_joint_target(angles, duration_sec)

    def reset_position(self, duration_sec: float = 2.0) -> bool:
        return self.send_joint_target(self.model.home_position.copy(), duration_sec)

    def solve_ik(
        self,
        target_position: List[float],
        target_rotation: Optional[List[List[float]]] = None,
        initial_joints: Optional[List[float]] = None,
        duration_hint: float = 0.5,
        keep_orientation: bool = False,
    ) -> Tuple[List[float], bool, float, float]:
        return self.kin.solve_ik(
            target_position=target_position,
            target_rotation=target_rotation,
            initial_joints=initial_joints,
            duration_hint=duration_hint,
            keep_orientation=keep_orientation,
        )

    def move_end_effector_world(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        duration: float = 0.8,
    ) -> bool:
        """
        Move TCP in Cartesian coordinates.

        Small manual movements are sent as one IK target.
        Longer movements are split into segments, so TCP motion is closer to
        a straight line.
        """
        try:
            if abs(dx) < 1e-12 and abs(dy) < 1e-12 and abs(dz) < 1e-12:
                return True

            q_start = self.get_current_joints()
            R_start, p_start = self.kin.forward_kinematics_full(q_start)

            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance < 0.006 or duration <= 0.15:
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
                self.send_joint_target(q_solution, duration)
                if not success:
                    self._warn(
                        "IK single move not fully converged: "
                        f"pos_error={pos_error:.6f}, rot_error={rot_error:.6f}"
                    )
                return success

            segment_length = 0.008
            segments = max(2, min(40, math.ceil(distance / segment_length)))
            q_current = q_start.copy()
            trajectory_points: List[List[float]] = []
            success_all = True
            max_pos_error = 0.0
            max_rot_error = 0.0

            for i in range(1, segments + 1):
                k = i / segments
                target_position = [
                    p_start[0] + dx * k,
                    p_start[1] + dy * k,
                    p_start[2] + dz * k,
                ]

                q_solution, success, pos_error, rot_error = self.solve_ik(
                    target_position=target_position,
                    target_rotation=R_start,
                    initial_joints=q_current,
                    duration_hint=duration / segments,
                    keep_orientation=True,
                )

                if not success:
                    success_all = False

                max_pos_error = max(max_pos_error, pos_error)
                max_rot_error = max(max_rot_error, rot_error)

                q_current = q_solution
                trajectory_points.append(q_current.copy())

            if not trajectory_points:
                return False

            delta_sum = sum(
                abs(trajectory_points[-1][i] - q_start[i])
                for i in range(self.model.joint_count)
            )

            if delta_sum < 1e-7:
                self._warn(
                    "Segmented move_lin produced almost no motion. "
                    "Fallback to single-step IK."
                )
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
                self.send_joint_target(q_solution, duration)
                if not success:
                    self._warn(
                        "Fallback IK not fully converged: "
                        f"pos_error={pos_error:.6f}, rot_error={rot_error:.6f}"
                    )
                return success

            self.send_joint_trajectory(trajectory_points, duration)

            if not success_all:
                self._warn(
                    "Segmented move_lin IK warning: "
                    f"max_pos_error={max_pos_error:.6f} m, "
                    f"max_rot_error={math.degrees(max_rot_error):.3f} deg"
                )

            return success_all

        except Exception as exc:
            self._error(f"move_end_effector_world error: {exc}")
            return False

    def rotate_end_effector_rx_ry_ik(
        self,
        d_rx: float = 0.0,
        d_ry: float = 0.0,
        duration: float = 0.1,
    ) -> bool:
        try:
            if abs(d_rx) < 1e-12 and abs(d_ry) < 1e-12:
                return True

            q_start = self.get_current_joints()
            R_start, p_start = self.kin.forward_kinematics_full(q_start)

            R_delta_x = self.kin.rot_x(d_rx)
            R_delta_y = self.kin.rot_y(d_ry)
            R_delta = self.kin.matmul3(R_delta_x, R_delta_y)
            R_target = self.kin.matmul3(R_start, R_delta)

            q_solution, success, pos_error, rot_error = self.solve_ik(
                target_position=p_start,
                target_rotation=R_target,
                initial_joints=q_start,
                duration_hint=duration,
                keep_orientation=False,
            )

            self.send_joint_target(q_solution, duration)

            if not success:
                self._warn(
                    "IK RX/RY rotation not fully converged: "
                    f"pos_error={pos_error:.6f}, rot_error={rot_error:.6f}"
                )

            return success

        except Exception as exc:
            self._error(f"rotate_end_effector_rx_ry_ik error: {exc}")
            return False

    def rotate_end_effector_world(
        self,
        drz: float = 0.0,
        duration: float = 0.1,
    ) -> bool:
        try:
            if abs(drz) < 1e-12:
                return True

            q_start = self.get_current_joints()
            R_start, p_start = self.kin.forward_kinematics_full(q_start)

            R_delta = self.kin.rot_z(drz)
            R_target = self.kin.matmul3(R_start, R_delta)

            q_solution, success, pos_error, rot_error = self.solve_ik(
                target_position=p_start,
                target_rotation=R_target,
                initial_joints=q_start,
                duration_hint=duration,
                keep_orientation=False,
            )

            self.send_joint_target(q_solution, duration)

            if not success:
                self._warn(
                    "IK RZ rotation not fully converged: "
                    f"pos_error={pos_error:.6f}, rot_error={rot_error:.6f}"
                )

            return success

        except Exception as exc:
            self._error(f"rotate_end_effector_world error: {exc}")
            return False

    def save_reference_orientation(self) -> None:
        R, _ = self.kin.forward_kinematics_full(self.get_current_joints())
        self.reference_orientation = R
        self._info("Reference TCP orientation saved.")

    def align_orientation_to_reference(self, duration_sec: float = 0.3) -> bool:
        if self.reference_orientation is None:
            self._warn("Reference orientation is not saved yet.")
            return False

        q_start = self.get_current_joints()
        _, p_current = self.kin.forward_kinematics_full(q_start)

        q_solution, success, pos_error, rot_error = self.solve_ik(
            target_position=p_current,
            target_rotation=self.reference_orientation,
            initial_joints=q_start,
            duration_hint=duration_sec,
            keep_orientation=False,
        )

        self.send_joint_target(q_solution, duration_sec)

        if not success:
            self._warn(
                "align_orientation_to_reference not fully converged: "
                f"pos_error={pos_error:.6f}, rot_error={rot_error:.6f}"
            )

        return success