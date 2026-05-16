import math
import threading
import time
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

from robot_arm_controller.core.gripper_controller import GripperController
from robot_arm_controller.core.motion_controller import MotionController
from robot_arm_controller.core.robot_model import RobotModel
from robot_arm_controller.core.urdf_kinematics import UrdfKinematics


class RobotBridge(Node):
    def __init__(self) -> None:
        super().__init__("robot_arm_controller")

        self.model = RobotModel()
        self.model.validate()

        self.kin = UrdfKinematics(self.model)
        self.lock = threading.RLock()

        self.current_position: List[float] = self.model.home_position.copy()
        self.current_gripper_position: List[float] = [0.0, 0.0]

        self.is_connected: bool = False
        self.has_joint_states: bool = False

        self.callbacks: List[Callable[[Dict], None]] = []

        self.speed_scale: float = 1.0

        self.last_arm_target: Optional[List[float]] = None
        self.last_gripper_target: Optional[float] = None

        self.motion_state: str = "idle"
        self.last_command_time: float = 0.0
        self.last_command_duration: float = 0.0

        self.e_stop_active: bool = False

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

        self.motion = MotionController(
            model=self.model,
            kinematics=self.kin,
            get_current_joints=self.get_current_position,
            publish_arm_points=self._publish_arm_points,
            logger=self.get_logger(),
        )

        self.gripper = GripperController(
            model=self.model,
            publish_gripper_target=self._publish_gripper_target,
            logger=self.get_logger(),
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.ros_thread = threading.Thread(target=self.spin_thread, daemon=True)
        self.ros_thread.start()

        self.is_connected = True

        self.get_logger().info(
            "RobotBridge started. "
            f"Arm topic={self.model.arm_topic}, joints={self.model.arm_joint_names}. "
            f"Gripper topic={self.model.gripper_topic}, joints={self.model.gripper_joint_names}."
        )

    def spin_thread(self) -> None:
        while rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.01)
            except Exception as exc:
                self.get_logger().error(f"ROS spin error: {exc}")

            time.sleep(0.05)

    def timer_callback(self) -> None:
        self._update_motion_state()

    def shutdown(self) -> None:
        self.is_connected = False

        try:
            self.destroy_node()
        except Exception as exc:
            self.get_logger().warn(f"destroy_node failed: {exc}")

    def joint_state_callback(self, msg: JointState) -> None:
        with self.lock:
            old_arm = self.current_position.copy()
            old_gripper = self.current_gripper_position.copy()

            arm_positions: List[float] = []

            for i, joint_name in enumerate(self.model.arm_joint_names):
                try:
                    idx = msg.name.index(joint_name)
                    arm_positions.append(float(msg.position[idx]))
                except (ValueError, IndexError):
                    arm_positions.append(old_arm[i])

            gripper_positions: List[float] = []

            for i, joint_name in enumerate(self.model.gripper_joint_names):
                try:
                    idx = msg.name.index(joint_name)
                    gripper_positions.append(float(msg.position[idx]))
                except (ValueError, IndexError):
                    if i < len(old_gripper):
                        gripper_positions.append(old_gripper[i])
                    else:
                        gripper_positions.append(0.0)

            self.current_position = arm_positions
            self.current_gripper_position = gripper_positions
            self.has_joint_states = True

        self.notify_callbacks(self.build_state_packet())

    def build_state_packet(self) -> Dict:
        data: Dict = {
            "position": self.get_current_position(),
            "gripper": self.get_current_gripper_position(),
            "status": self.get_status(),
        }

        try:
            data["pose"] = self.get_tcp_pose()
        except Exception as exc:
            self.get_logger().error(f"get_tcp_pose error: {exc}")

        return data

    def get_status(self) -> Dict:
        with self.lock:
            return {
                "connected": self.is_connected,
                "joint_states": self.has_joint_states,
                "motion_state": self.motion_state,
                "e_stop_active": self.e_stop_active,
                "arm_target": (
                    self.last_arm_target.copy()
                    if self.last_arm_target is not None
                    else None
                ),
                "gripper_target": self.last_gripper_target,
            }

    @staticmethod
    def duration_from_float(duration_sec: float) -> Duration:
        duration_sec = max(0.0, float(duration_sec))
        total_nanoseconds = int(duration_sec * 1_000_000_000)
        sec = total_nanoseconds // 1_000_000_000
        nanosec = total_nanoseconds % 1_000_000_000

        return Duration(sec=sec, nanosec=nanosec)

    def _publish_arm_points(
        self,
        points: List[List[float]],
        duration_sec: float,
        force: bool = False,
    ) -> bool:
        if not force and self.e_stop_active:
            self.get_logger().warn("Arm command blocked because Emergency Stop is active.")
            return False

        if not points:
            self.get_logger().warn("_publish_arm_points called with empty points")
            return False

        total_duration = max(0.05, float(duration_sec))

        traj = JointTrajectory()
        traj.joint_names = self.model.arm_joint_names.copy()

        for i, q_in in enumerate(points, start=1):
            if len(q_in) != self.model.joint_count:
                self.get_logger().error(
                    f"Trajectory point expected {self.model.joint_count} values, got {len(q_in)}"
                )
                return False

            q = self.kin.clamp_to_joint_limits([float(value) for value in q_in])

            point = JointTrajectoryPoint()
            point.positions = q
            point.time_from_start = self.duration_from_float(
                total_duration * i / len(points)
            )

            traj.points.append(point)

        final_target = list(traj.points[-1].positions)

        with self.lock:
            self.last_arm_target = final_target
            self.last_command_time = time.monotonic()
            self.last_command_duration = total_duration
            self.motion_state = "moving"

        self.get_logger().info(
            "Publishing arm trajectory: "
            f"points={len(traj.points)}, "
            f"final={[round(v, 4) for v in final_target]}, "
            f"duration={total_duration:.2f}s"
        )

        self.trajectory_publisher.publish(traj)
        self.notify_callbacks(self.build_state_packet())

        return True

    def _publish_gripper_target(
        self,
        opening: float,
        duration_sec: float,
        force: bool = False,
    ) -> bool:
        if not force and self.e_stop_active:
            self.get_logger().warn("Gripper command blocked because Emergency Stop is active.")
            return False

        opening = self.model.clamp_gripper_opening(opening)
        duration_sec = max(0.05, float(duration_sec))

        traj = JointTrajectory()
        traj.joint_names = self.model.gripper_joint_names.copy()

        point = JointTrajectoryPoint()
        point.positions = [opening for _ in self.model.gripper_joint_names]
        point.time_from_start = self.duration_from_float(duration_sec)

        traj.points.append(point)

        with self.lock:
            self.last_gripper_target = opening

        self.get_logger().info(
            f"Publishing gripper trajectory: opening={opening:.4f} m, duration={duration_sec:.2f}s"
        )

        self.gripper_publisher.publish(traj)
        self.notify_callbacks(self.build_state_packet())

        return True

    def _update_motion_state(self) -> None:
        with self.lock:
            if self.e_stop_active:
                self.motion_state = "estop"
                return

            target = self.last_arm_target.copy() if self.last_arm_target else None
            current = self.current_position.copy()
            started_at = self.last_command_time
            planned_duration = self.last_command_duration
            state = self.motion_state

        if target is None or state not in ("moving", "timeout"):
            return

        error = max(
            abs(current[i] - target[i])
            for i in range(self.model.joint_count)
        )

        if error <= self.model.joint_goal_tolerance:
            with self.lock:
                self.motion_state = "done"

            self.notify_callbacks(self.build_state_packet())
            return

        elapsed = time.monotonic() - started_at
        timeout = planned_duration + self.model.motion_timeout_margin

        if elapsed > timeout:
            with self.lock:
                self.motion_state = "timeout"

            self.notify_callbacks(self.build_state_packet())

    def wait_until_arm_reached(
        self,
        timeout_sec: Optional[float] = None,
        tolerance: Optional[float] = None,
    ) -> bool:
        tolerance = tolerance or self.model.joint_goal_tolerance

        with self.lock:
            target = self.last_arm_target.copy() if self.last_arm_target else None
            duration = self.last_command_duration

        if target is None:
            return True

        if timeout_sec is None:
            timeout_sec = duration + self.model.motion_timeout_margin

        start = time.monotonic()

        while time.monotonic() - start <= timeout_sec:
            if self.e_stop_active:
                return False

            current = self.get_current_position()

            error = max(
                abs(current[i] - target[i])
                for i in range(self.model.joint_count)
            )

            if error <= tolerance:
                with self.lock:
                    self.motion_state = "done"

                return True

            time.sleep(0.02)

        with self.lock:
            self.motion_state = "timeout"

        return False

    def stop_motion(self) -> bool:
        current = self.get_current_position()

        with self.lock:
            self.motion_state = "stopping"

        return self._publish_arm_points([current], 0.1, force=True)

    def emergency_stop(self) -> None:
        self.stop_motion()

        with self.lock:
            self.e_stop_active = True
            self.motion_state = "estop"

        self.get_logger().warn("Emergency Stop activated.")
        self.notify_callbacks(self.build_state_packet())

    def reset_emergency_stop(self) -> None:
        with self.lock:
            self.e_stop_active = False
            self.motion_state = "idle"

        self.get_logger().info("Emergency Stop reset.")
        self.notify_callbacks(self.build_state_packet())

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
        return self.motion.solve_ik(
            target_position=target_position,
            target_rotation=target_rotation,
            initial_joints=initial_joints,
            duration_hint=duration_hint,
            keep_orientation=keep_orientation,
        )

    def send_trajectory(
        self,
        joint_angles: List[float],
        duration_sec: float = 1.0,
    ) -> bool:
        return self.motion.send_joint_target(joint_angles, duration_sec)

    def send_trajectory_points(
        self,
        points: List[List[float]],
        duration_sec: float,
    ) -> bool:
        return self.motion.send_joint_trajectory(points, duration_sec)

    def move_end_effector_world(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        duration: float = 0.8,
    ) -> bool:
        return self.motion.move_end_effector_world(dx, dy, dz, duration)

    def rotate_end_effector_rx_ry_ik(
        self,
        d_rx: float = 0.0,
        d_ry: float = 0.0,
        duration: float = 0.1,
        **kwargs,
    ) -> bool:
        if "drx" in kwargs:
            d_rx = kwargs["drx"]

        if "dry" in kwargs:
            d_ry = kwargs["dry"]

        return self.motion.rotate_end_effector_rx_ry_ik(d_rx, d_ry, duration)

    def rotate_end_effector_world(
        self,
        drz: float = 0.0,
        duration: float = 0.1,
    ) -> bool:
        return self.motion.rotate_end_effector_world(drz, duration)

    def move_joint(
        self,
        joint_index: int,
        angle: float,
        duration_sec: float = 0.5,
    ) -> bool:
        return self.motion.move_joint(joint_index, angle, duration_sec)

    def move_joints_absolute(
        self,
        angles: List[float],
        duration_sec: float = 1.0,
    ) -> bool:
        return self.motion.move_joints_absolute(angles, duration_sec)

    def reset_position(self) -> bool:
        return self.motion.reset_position(2.0)

    def open_gripper(self, duration_sec: float = 0.7) -> bool:
        return self.gripper.open(duration_sec)

    def close_gripper(self, duration_sec: float = 0.7) -> bool:
        return self.gripper.close(duration_sec)

    def set_gripper(self, opening: float, duration_sec: float = 0.7) -> bool:
        return self.gripper.set_opening(opening, duration_sec)

    def send_gripper_trajectory(
        self,
        opening: float,
        duration_sec: float = 0.7,
    ) -> bool:
        return self.gripper.set_opening(opening, duration_sec)

    def save_reference_orientation(self) -> None:
        self.motion.save_reference_orientation()

    def align_orientation_to_reference(self, duration_sec: float = 0.3) -> bool:
        return self.motion.align_orientation_to_reference(duration_sec)

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
        rotation, _ = self.forward_kinematics_full(self.get_current_position())
        return self.kin.quaternion_from_rotation_matrix(rotation)

    def get_tcp_pose(self) -> Dict[str, float]:
        joints = self.get_current_position()
        rotation, position = self.forward_kinematics_full(joints)

        sy = math.sqrt(
            rotation[0][0] * rotation[0][0]
            + rotation[1][0] * rotation[1][0]
        )

        singular = sy < 1e-6

        if not singular:
            rx = math.atan2(rotation[2][1], rotation[2][2])
            ry = math.atan2(-rotation[2][0], sy)
            rz = math.atan2(rotation[1][0], rotation[0][0])
        else:
            rx = math.atan2(-rotation[1][2], rotation[1][1])
            ry = math.atan2(-rotation[2][0], sy)
            rz = 0.0

        return {
            "x": -position[0],
            "y": -position[1],
            "z": position[2],
            "rx": math.degrees(rx),
            "ry": math.degrees(ry),
            "rz": math.degrees(rz),
        }

    def check_fk_against_tf(self) -> Optional[Dict[str, float]]:
        if self.tf_buffer is None:
            self.get_logger().warn("TF buffer is not available.")
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                self.model.base_frame,
                self.model.tool_frame,
                Time(),
            )

            tf_x = transform.transform.translation.x
            tf_y = transform.transform.translation.y
            tf_z = transform.transform.translation.z

            _, fk_position = self.forward_kinematics_full(self.get_current_position())

            fk_x = fk_position[0]
            fk_y = fk_position[1]
            fk_z = fk_position[2]

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

    def register_callback(self, callback: Callable[[Dict], None]) -> None:
        self.callbacks.append(callback)

    def notify_callbacks(self, data: Dict) -> None:
        for callback in self.callbacks:
            try:
                callback(data)
            except Exception as exc:
                self.get_logger().warn(f"GUI callback error: {exc}")