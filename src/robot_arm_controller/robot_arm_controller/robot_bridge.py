import math
import threading
from typing import Callable, Dict, List, Optional

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    import tf2_ros
except ImportError:
    tf2_ros = None

from robot_arm_controller.core.robot_model import RobotModel
from robot_arm_controller.core.urdf_kinematics import UrdfKinematics


class RobotBridge(Node):
    """
    ROS 2 bridge между GUI и Gazebo.

    Core-рефакторинг эталонной рабочей версии:
    - ROS publishers/subscribers здесь;
    - модель робота в core/robot_model.py;
    - FK/IK в core/urdf_kinematics.py;
    - движение сохранено близко к рабочей версии;
    - move_lin улучшен через сегментированную траекторию;
    - добавлена инженерная проверка FK против /tf.
    """

    def __init__(self):
        super().__init__("robot_arm_controller_v4")

        self.model = RobotModel()
        self.kin = UrdfKinematics(self.model)

        self.tf_buffer = None
        self.tf_listener = None

        if tf2_ros is not None:
            try:
                self.tf_buffer = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
                self.get_logger().info("TF listener initialized.")
            except Exception as exc:
                self.get_logger().warn(f"TF listener initialization failed: {exc}")
        else:
            self.get_logger().warn("tf2_ros is not available. FK/TF check disabled.")

        self.current_position: List[float] = [0.0] * 6
        self.current_gripper_position: List[float] = [0.0, 0.0]

        self.is_connected: bool = False
        self.callbacks: List[Callable[[Dict], None]] = []
        self.lock = threading.Lock()

        self.speed_scale = 1.0
        self.reference_orientation: Optional[List[List[float]]] = None

        self.last_arm_target: Optional[List[float]] = None
        self.last_gripper_target: Optional[float] = None

        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            self.model.arm_topic,
            10,
        )

        self.gripper_publisher = self.create_publisher(
            JointTrajectory,
            self.model.gripper_topic,
            10,
        )

        self.state_subscriber = self.create_subscription(
            JointState,
            self.model.joint_states_topic,
            self.joint_state_callback,
            10,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.ros_thread = threading.Thread(target=self.spin_thread, daemon=True)
        self.ros_thread.start()

        self.is_connected = True

        self.get_logger().info(
            "RobotBridge core version started. "
            f"Arm topic={self.model.arm_topic}, joints={self.model.arm_joint_names}. "
            f"Gripper topic={self.model.gripper_topic}, joints={self.model.gripper_joint_names}."
        )

    # ------------------------------------------------------------------
    # ROS
    # ------------------------------------------------------------------

    def spin_thread(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def timer_callback(self):
        pass

    def joint_state_callback(self, msg: JointState):
        """
        Получение фактических положений суставов из /joint_states.
        """

        with self.lock:
            old_arm = self.current_position.copy()
            old_gripper = self.current_gripper_position.copy()

        arm_positions: List[float] = []

        for i, joint_name in enumerate(self.model.arm_joint_names):
            try:
                idx = msg.name.index(joint_name)
                arm_positions.append(float(msg.position[idx]))
            except ValueError:
                arm_positions.append(old_arm[i])

        gripper_positions: List[float] = []

        for i, joint_name in enumerate(self.model.gripper_joint_names):
            try:
                idx = msg.name.index(joint_name)
                gripper_positions.append(float(msg.position[idx]))
            except ValueError:
                gripper_positions.append(old_gripper[i])

        with self.lock:
            self.current_position = arm_positions
            self.current_gripper_position = gripper_positions

        data = {
            "position": arm_positions,
            "gripper": gripper_positions,
            "status": {
                "connected": self.is_connected,
                "arm_target": self.last_arm_target,
                "gripper_target": self.last_gripper_target,
            },
        }

        try:
            data["pose"] = self.get_tcp_pose()
        except Exception as exc:
            self.get_logger().error(f"get_tcp_pose error in joint_state_callback: {exc}")

        self.notify_callbacks(data)

    @staticmethod
    def duration_from_float(duration_sec: float) -> Duration:
        duration_sec = max(0.0, float(duration_sec))
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)

        return Duration(sec=sec, nanosec=nanosec)

    # ------------------------------------------------------------------
    # FK / IK wrappers
    # ------------------------------------------------------------------

    def forward_kinematics(self, joints: List[float]):
        return self.kin.forward_kinematics(joints)

    def forward_kinematics_full(self, joints: List[float]):
        return self.kin.forward_kinematics_full(joints)

    def compute_position_jacobian(self, joints: List[float]):
        return self.kin.compute_position_jacobian(joints)

    def compute_orientation_jacobian(self, joints: List[float]):
        return self.kin.compute_orientation_jacobian(joints)

    def solve_ik(
        self,
        target_position: List[float],
        target_rotation: Optional[List[List[float]]] = None,
        initial_joints: Optional[List[float]] = None,
        duration_hint: float = 0.5,
        keep_orientation: bool = False,
    ):
        return self.kin.solve_ik(
            target_position=target_position,
            target_rotation=target_rotation,
            initial_joints=initial_joints,
            duration_hint=duration_hint,
            keep_orientation=keep_orientation,
        )

    # ------------------------------------------------------------------
    # Arm trajectory
    # ------------------------------------------------------------------

    def send_trajectory(
        self,
        joint_angles: List[float],
        duration_sec: float = 1.0,
    ):
        """
        Отправка одной точки траектории в arm_controller.

        ВАЖНО:
        header.stamp не задаём. Нулевой timestamp означает "выполнять сразу".
        """

        if len(joint_angles) != 6:
            self.get_logger().error(
                f"send_trajectory expected 6 joint angles, got {len(joint_angles)}"
            )
            return

        q = self.kin.clamp_to_joint_limits([float(v) for v in joint_angles])

        traj = JointTrajectory()
        traj.joint_names = self.model.arm_joint_names.copy()

        point = JointTrajectoryPoint()
        point.positions = q
        point.time_from_start = self.duration_from_float(duration_sec)

        traj.points.append(point)

        self.last_arm_target = q.copy()

        self.get_logger().info(
            "Publishing arm trajectory: "
            f"positions={[round(v, 4) for v in q]}, "
            f"duration={duration_sec:.2f}s"
        )

        self.trajectory_publisher.publish(traj)

    def send_trajectory_points(
        self,
        points: List[List[float]],
        duration_sec: float,
    ):
        """
        Отправка многоточечной траектории в arm_controller.

        Используется для сегментированного move_lin:
        вместо одной конечной точки отправляется несколько промежуточных
        суставных конфигураций, поэтому TCP движется ближе к прямой.
        """

        if not points:
            return

        traj = JointTrajectory()
        traj.joint_names = self.model.arm_joint_names.copy()

        total = max(0.05, float(duration_sec))

        for i, q in enumerate(points, start=1):
            if len(q) != 6:
                self.get_logger().error(
                    f"Trajectory point expected 6 joint values, got {len(q)}"
                )
                return

            q = self.kin.clamp_to_joint_limits(q)

            point = JointTrajectoryPoint()
            point.positions = q

            t = total * i / len(points)
            point.time_from_start = self.duration_from_float(t)

            traj.points.append(point)

        self.last_arm_target = list(traj.points[-1].positions)

        self.get_logger().info(
            "Publishing segmented arm trajectory: "
            f"points={len(traj.points)}, "
            f"final={[round(v, 4) for v in self.last_arm_target]}, "
            f"duration={duration_sec:.2f}s"
        )

        self.trajectory_publisher.publish(traj)

    # ------------------------------------------------------------------
    # Cartesian movement
    # ------------------------------------------------------------------

    def move_end_effector_world(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        duration: float = 0.8,
    ) -> bool:
        """
        Сегментированное линейное смещение TCP.

        Логика:
        - для маленьких ручных шагов используется одиночная IK-команда;
        - для длинных move_lin движение разбивается на промежуточные точки;
        - ориентация TCP сохраняется как в эталонной рабочей версии;
        - если сегментированный расчёт почти не дал движения, используется fallback
          на старую одиночную IK-команду.
        """

        try:
            if abs(dx) < 1e-12 and abs(dy) < 1e-12 and abs(dz) < 1e-12:
                return True

            with self.lock:
                q_start = self.current_position.copy()

            R_start, p_start = self.forward_kinematics_full(q_start)

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

                self.send_trajectory(q_solution, duration)

                if not success:
                    self.get_logger().warn(
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
                for i in range(6)
            )

            if delta_sum < 1e-7:
                self.get_logger().warn(
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

                self.send_trajectory(q_solution, duration)

                if not success:
                    self.get_logger().warn(
                        "Fallback IK not fully converged: "
                        f"pos_error={pos_error:.6f}, rot_error={rot_error:.6f}"
                    )

                return success

            self.send_trajectory_points(trajectory_points, duration)

            if not success_all:
                self.get_logger().warn(
                    "Segmented move_lin IK warning: "
                    f"max_pos_error={max_pos_error:.6f} m, "
                    f"max_rot_error={math.degrees(max_rot_error):.3f} deg"
                )

            return success_all

        except Exception as exc:
            self.get_logger().error(f"move_end_effector_world error: {exc}")
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

            with self.lock:
                q_start = self.current_position.copy()

            R_start, p_start = self.forward_kinematics_full(q_start)

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
        try:
            if abs(drz) < 1e-12:
                return True

            with self.lock:
                q_start = self.current_position.copy()

            R_start, p_start = self.forward_kinematics_full(q_start)

            R_delta = self.kin.rot_z(drz)
            R_target = self.kin.matmul3(R_start, R_delta)

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

    # ------------------------------------------------------------------
    # Joint movement
    # ------------------------------------------------------------------

    def move_joint(
        self,
        joint_index: int,
        angle: float,
        duration_sec: float = 0.5,
    ):
        """
        Управление отдельным суставом.

        Поддерживаются:
        - joint_index = 1..6 для program editor;
        - joint_index = 0..5 для внутреннего вызова.
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
        q = self.kin.clamp_to_joint_limits(q)

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
    # Gripper
    # ------------------------------------------------------------------

    def send_gripper_trajectory(
        self,
        opening: float,
        duration_sec: float = 0.7,
    ):
        opening = self.kin.clamp(
            float(opening),
            self.model.gripper_closed_position,
            self.model.gripper_open_position,
        )

        traj = JointTrajectory()
        traj.joint_names = self.model.gripper_joint_names.copy()

        point = JointTrajectoryPoint()
        point.positions = [opening, opening]
        point.time_from_start = self.duration_from_float(duration_sec)

        traj.points.append(point)

        self.last_gripper_target = opening

        self.get_logger().info(
            "Publishing gripper trajectory: "
            f"opening={opening:.4f} m, duration={duration_sec:.2f}s"
        )

        self.gripper_publisher.publish(traj)

    def open_gripper(self, duration_sec: float = 0.7):
        self.send_gripper_trajectory(
            self.model.gripper_open_position,
            duration_sec,
        )

    def close_gripper(self, duration_sec: float = 0.7):
        self.send_gripper_trajectory(
            self.model.gripper_closed_position,
            duration_sec,
        )

    def set_gripper(self, opening: float, duration_sec: float = 0.7):
        self.send_gripper_trajectory(opening, duration_sec)

    # ------------------------------------------------------------------
    # Reference orientation
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
    # State
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
        return self.kin.quaternion_from_rotation_matrix(R)

    def get_tcp_pose(self) -> Dict[str, float]:
        q = self.get_current_position()
        R, p = self.forward_kinematics_full(q)

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

        return {
            "x": -p[0],
            "y": -p[1],
            "z": p[2],
            "rx": math.degrees(rx),
            "ry": math.degrees(ry),
            "rz": math.degrees(rz),
        }

    # ------------------------------------------------------------------
    # FK / TF verification
    # ------------------------------------------------------------------

    def check_fk_against_tf(self) -> Optional[Dict[str, float]]:
        """
        Сравнение Python FK с реальным TF base_link -> tool0.

        FK Python показывает, что считает наша математическая модель.
        TF показывает, что реально публикует robot_state_publisher.
        Разница показывает ошибку модели.
        """

        if self.tf_buffer is None:
            self.get_logger().warn("TF buffer is not available.")
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                "tool0",
                Time(),
            )

            tf_x = transform.transform.translation.x
            tf_y = transform.transform.translation.y
            tf_z = transform.transform.translation.z

            with self.lock:
                q = self.current_position.copy()

            _, fk_p = self.forward_kinematics_full(q)

            fk_x = fk_p[0]
            fk_y = fk_p[1]
            fk_z = fk_p[2]

            dx = fk_x - tf_x
            dy = fk_y - tf_y
            dz = fk_z - tf_z

            error_m = math.sqrt(dx * dx + dy * dy + dz * dz)

            result = {
                "fk_x": fk_x,
                "fk_y": fk_y,
                "fk_z": fk_z,
                "tf_x": tf_x,
                "tf_y": tf_y,
                "tf_z": tf_z,
                "error_x_mm": dx * 1000.0,
                "error_y_mm": dy * 1000.0,
                "error_z_mm": dz * 1000.0,
                "error_total_mm": error_m * 1000.0,
            }

            self.get_logger().info(
                "FK/TF check: "
                f"FK=({fk_x:.4f}, {fk_y:.4f}, {fk_z:.4f}) m, "
                f"TF=({tf_x:.4f}, {tf_y:.4f}, {tf_z:.4f}) m, "
                f"error=({result['error_x_mm']:.2f}, "
                f"{result['error_y_mm']:.2f}, "
                f"{result['error_z_mm']:.2f}) mm, "
                f"total={result['error_total_mm']:.2f} mm"
            )

            return result

        except Exception as exc:
            self.get_logger().warn(f"FK/TF check failed: {exc}")
            return None

    # ------------------------------------------------------------------
    # GUI callbacks
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