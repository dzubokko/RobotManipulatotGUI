import math
import os
import signal
import subprocess
import threading
from typing import Callable, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class RobotBridge(Node):
    """
    Стабильный мост между GUI и твоим роботом.

    Что делает эта версия:
    - управляет joint_1 ... joint_6 через /arm_controller/joint_trajectory;
    - управляет захватом через /gripper_controller/joint_trajectory;
    - читает /joint_states;
    - запускает/останавливает Gazebo из GUI;
    - НЕ использует самописный IK для X/Y/Z.

    Почему Cartesian IK отключён:
    текущий самописный IK нестабилен для твоей URDF-цепочки.
    Для нормального X/Y/Z нужно подключать MoveIt 2 / MoveIt Servo.
    """

    ARM_JOINT_NAMES = [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    ]

    # У тебя right_finger_joint работает через mimic,
    # поэтому gripper_controller должен управлять только left_finger_joint.
    GRIPPER_COMMAND_JOINT_NAMES = [
        "left_finger_joint",
    ]

    GRIPPER_STATE_JOINT_NAMES = [
        "left_finger_joint",
        "right_finger_joint",
    ]

    ARM_TRAJECTORY_TOPIC = "/arm_controller/joint_trajectory"
    GRIPPER_TRAJECTORY_TOPIC = "/gripper_controller/joint_trajectory"

    HOME_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    GRIPPER_CLOSED_POSITION = 0.0
    GRIPPER_OPEN_POSITION = 0.02
    GRIPPER_MAX_POSITION = 0.025

    JOINT_LIMITS = [
        (-3.14, 3.14),        # joint_1
        (-0.7854, 1.5708),    # joint_2
        (-1.5708, 1.5708),    # joint_3
        (-3.1416, 3.1416),    # joint_4
        (-2.0944, 2.0944),    # joint_5
        (-3.14, 3.14),        # joint_6
    ]

    # Только для отображения TCP, не для управления.
    # Позже TCP должен прийти из MoveIt/TF.
    KINEMATIC_CHAIN = [
        {
            "name": "joint_1",
            "xyz": [0.0, 0.0, 0.1038],
            "rpy": [0.0, 0.0, 3.1416],
            "axis": [0.0, 0.0, 1.0],
        },
        {
            "name": "joint_2",
            "xyz": [-0.04375, -0.0558, 0.08],
            "rpy": [1.5708, 0.0, 0.0],
            "axis": [0.0, 0.0, 1.0],
        },
        {
            "name": "joint_3",
            "xyz": [0.0, 0.22, -0.04],
            "rpy": [0.0, 3.1416, 0.0],
            "axis": [0.0, 0.0, 1.0],
        },
        {
            "name": "joint_4",
            "xyz": [0.07, 0.075, 0.06],
            "rpy": [0.0, 1.5708, 0.0],
            "axis": [0.0, 0.0, 1.0],
        },
        {
            "name": "joint_5",
            "xyz": [0.0, 0.0, 0.172],
            "rpy": [1.5708, 0.0, -1.5708],
            "axis": [0.0, 0.0, 1.0],
        },
        {
            "name": "joint_6",
            "xyz": [0.0, 0.08593, 0.0],
            "rpy": [-1.5708, 0.0, 0.0],
            "axis": [0.0, 0.0, 1.0],
        },
    ]

    TOOL0_XYZ = [0.0, 0.0, 0.0867]
    TOOL0_RPY = [0.0, 0.0, 0.0]

    def __init__(self):
        super().__init__("robot_arm_controller_v4")

        self.current_position: List[float] = [0.0] * 6
        self.current_gripper_position: List[float] = [0.0, 0.0]

        self.is_connected: bool = False
        self.callbacks: List[Callable[[Dict], None]] = []
        self.lock = threading.Lock()

        self.gazebo_process: Optional[subprocess.Popen] = None

        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            self.ARM_TRAJECTORY_TOPIC,
            10,
        )

        self.gripper_publisher = self.create_publisher(
            JointTrajectory,
            self.GRIPPER_TRAJECTORY_TOPIC,
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
        self.get_logger().info("RobotBridge connected to custom robot")

    # ============================================================
    # ROS
    # ============================================================

    def spin_thread(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def timer_callback(self):
        pass

    def joint_state_callback(self, msg: JointState):
        joint_state_map = dict(zip(msg.name, msg.position))

        positions = [
            float(joint_state_map.get(joint_name, 0.0))
            for joint_name in self.ARM_JOINT_NAMES
        ]

        gripper_positions = [
            float(joint_state_map.get(joint_name, 0.0))
            for joint_name in self.GRIPPER_STATE_JOINT_NAMES
        ]

        with self.lock:
            self.current_position = positions
            self.current_gripper_position = gripper_positions

        self.notify_callbacks(
            {
                "position": positions,
                "gripper_position": gripper_positions,
                "pose": self.get_tcp_pose(),
                "simulation_running": self.is_simulation_running(),
            }
        )

    # ============================================================
    # Simulation launch
    # ============================================================

    def start_simulation(self) -> bool:
        """
        Запускает Gazebo из GUI.
        Не запускает второй Gazebo, если один уже работает.
        """

        if self.is_simulation_running():
            self.get_logger().info("Gazebo is already running")
            return True

        cmd = (
            "cd ~/RobotManipulator && "
            "source /opt/ros/humble/setup.bash && "
            "source install/setup.bash && "
            "ros2 launch robot_gazebo gazebo.launch.py"
        )

        try:
            self.gazebo_process = subprocess.Popen(
                ["bash", "-lc", cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )

            self.get_logger().info("Gazebo launch started from GUI")
            return True

        except Exception as exc:
            self.get_logger().error(f"Failed to start Gazebo: {exc}")
            return False

    def stop_simulation(self) -> bool:
        """
        Останавливает Gazebo, если он был запущен из GUI.
        """

        try:
            if self.gazebo_process is not None and self.gazebo_process.poll() is None:
                os.killpg(os.getpgid(self.gazebo_process.pid), signal.SIGTERM)
                self.gazebo_process = None
                self.get_logger().info("Gazebo stopped from GUI")
                return True

            self.get_logger().info("No GUI-started Gazebo process to stop")
            return True

        except Exception as exc:
            self.get_logger().error(f"Failed to stop Gazebo: {exc}")
            return False

    def is_simulation_running(self) -> bool:
        if self.gazebo_process is not None and self.gazebo_process.poll() is None:
            return True

        return False

    # ============================================================
    # Helpers
    # ============================================================

    @staticmethod
    def _duration_msg(duration_sec: float) -> Duration:
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        return Duration(sec=sec, nanosec=nanosec)

    def clamp_joints(self, joints: List[float]) -> List[float]:
        result = []

        for i, value in enumerate(joints[:6]):
            lower, upper = self.JOINT_LIMITS[i]
            result.append(max(lower, min(upper, float(value))))

        while len(result) < 6:
            result.append(0.0)

        return result

    # ============================================================
    # Joint control
    # ============================================================

    def send_trajectory(self, joint_angles: List[float], duration_sec: float = 1.0):
        if len(joint_angles) != 6:
            self.get_logger().error(
                f"Expected 6 joint angles, got {len(joint_angles)}"
            )
            return

        joint_angles = self.clamp_joints(joint_angles)

        traj = JointTrajectory()
        traj.joint_names = self.ARM_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in joint_angles]
        point.time_from_start = self._duration_msg(duration_sec)

        traj.points.append(point)
        self.trajectory_publisher.publish(traj)

    def move_joint(self, joint_index: int, angle: float, duration_sec: float = 0.5):
        if joint_index < 0 or joint_index >= 6:
            self.get_logger().error(f"Invalid joint index: {joint_index}")
            return

        with self.lock:
            new_position = list(self.current_position)

        new_position[joint_index] = float(angle)
        self.send_trajectory(new_position, duration_sec)

    def move_joints_absolute(self, angles: List[float], duration_sec: float = 1.0):
        self.send_trajectory(angles, duration_sec)

    def reset_position(self):
        self.send_trajectory(self.HOME_POSITION, 2.0)

    def get_current_position(self) -> List[float]:
        with self.lock:
            return self.current_position.copy()

    # ============================================================
    # Gripper control
    # ============================================================

    def send_gripper_trajectory(self, position: float, duration_sec: float = 1.0):
        position = max(0.0, min(self.GRIPPER_MAX_POSITION, float(position)))

        traj = JointTrajectory()
        traj.joint_names = self.GRIPPER_COMMAND_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = self._duration_msg(duration_sec)

        traj.points.append(point)
        self.gripper_publisher.publish(traj)

    def open_gripper(self):
        self.send_gripper_trajectory(self.GRIPPER_OPEN_POSITION, 1.0)

    def close_gripper(self):
        self.send_gripper_trajectory(self.GRIPPER_CLOSED_POSITION, 1.0)

    def get_current_gripper_position(self) -> List[float]:
        with self.lock:
            return self.current_gripper_position.copy()

    # ============================================================
    # Cartesian movement placeholders
    # ============================================================

    def move_end_effector_world(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        duration: float = 0.8,
    ) -> bool:
        """
        Временно отключено.

        Почему:
        самописный IK в GUI даёт нестабильное движение.
        Правильный вариант — MoveIt 2 / MoveIt Servo.
        """

        self.get_logger().warn(
            "Cartesian XYZ movement is disabled. Use MoveIt 2 / MoveIt Servo for stable Cartesian control."
        )
        return False

    def rotate_end_effector_rx_ry_ik(
        self,
        d_rx: float = 0.0,
        d_ry: float = 0.0,
        duration: float = 0.1,
    ) -> bool:
        self.get_logger().warn(
            "Cartesian RX/RY rotation is disabled. Use MoveIt 2 / MoveIt Servo for stable Cartesian control."
        )
        return False

    def rotate_end_effector_world(
        self,
        drz: float = 0.0,
        duration: float = 0.1,
    ) -> bool:
        """
        Вращение вокруг последнего сустава оставляем, потому что это joint-space.
        """

        try:
            if abs(drz) < 1e-9:
                return True

            with self.lock:
                joints = list(self.current_position)

            joints[5] += float(drz)
            joints = self.clamp_joints(joints)

            self.send_trajectory(joints, duration)
            return True

        except Exception as exc:
            self.get_logger().error(f"rotate_end_effector_world failed: {exc}")
            return False

    # ============================================================
    # FK for display only
    # ============================================================

    @staticmethod
    def _matmul_4x4(A, B):
        result = [[0.0] * 4 for _ in range(4)]

        for i in range(4):
            for j in range(4):
                result[i][j] = sum(A[i][k] * B[k][j] for k in range(4))

        return result

    @staticmethod
    def _translation_matrix(xyz: List[float]):
        x, y, z = xyz

        return [
            [1.0, 0.0, 0.0, x],
            [0.0, 1.0, 0.0, y],
            [0.0, 0.0, 1.0, z],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _rpy_matrix(rpy: List[float]):
        roll, pitch, yaw = rpy

        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        return [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.0],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.0],
            [-sp, cp * sr, cp * cr, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _axis_angle_matrix(axis: List[float], angle: float):
        x, y, z = axis
        norm = math.sqrt(x * x + y * y + z * z)

        if norm < 1e-12:
            return [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]

        x /= norm
        y /= norm
        z /= norm

        c = math.cos(angle)
        s = math.sin(angle)
        C = 1.0 - c

        return [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s, 0.0],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s, 0.0],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @classmethod
    def _origin_matrix(cls, xyz: List[float], rpy: List[float]):
        T = cls._translation_matrix(xyz)
        R = cls._rpy_matrix(rpy)
        return cls._matmul_4x4(T, R)

    def forward_kinematics_full(
        self,
        joints: List[float],
    ) -> Tuple[List[List[float]], List[float]]:
        if len(joints) < 6:
            joints = list(joints) + [0.0] * (6 - len(joints))

        joints = self.clamp_joints(joints)

        T = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

        for i, joint_data in enumerate(self.KINEMATIC_CHAIN):
            T_origin = self._origin_matrix(
                joint_data["xyz"],
                joint_data["rpy"],
            )

            T_motion = self._axis_angle_matrix(
                joint_data["axis"],
                joints[i],
            )

            T = self._matmul_4x4(T, T_origin)
            T = self._matmul_4x4(T, T_motion)

        T_tool_origin = self._origin_matrix(self.TOOL0_XYZ, self.TOOL0_RPY)
        T = self._matmul_4x4(T, T_tool_origin)

        R = [
            [T[0][0], T[0][1], T[0][2]],
            [T[1][0], T[1][1], T[1][2]],
            [T[2][0], T[2][1], T[2][2]],
        ]

        p = [T[0][3], T[1][3], T[2][3]]

        return R, p

    def forward_kinematics(self, joints: List[float]) -> Tuple[float, float, float]:
        _, p = self.forward_kinematics_full(joints)
        return p[0], p[1], p[2]

    def get_tcp_position(self) -> List[float]:
        x, y, z = self.forward_kinematics(self.get_current_position())
        return [x, y, z]

    def get_tcp_pose(self) -> Dict[str, float]:
        with self.lock:
            q = list(self.current_position)

        x, y, z = self.forward_kinematics(q)

        return {
            "x": x,
            "y": y,
            "z": z,
            "rx": 0.0,
            "ry": 0.0,
            "rz": 0.0,
        }

    def get_tcp_orientation(self) -> List[float]:
        return [0.0, 0.0, 0.0, 1.0]

    def save_reference_orientation(self):
        self.get_logger().info("Reference orientation is disabled in stable bridge")

    def align_orientation_to_reference(self, duration: float = 0.3):
        self.get_logger().warn("Reference orientation alignment is disabled")
        return False

    def save_ref(self):
        self.save_reference_orientation()

    def align_to_ref(self):
        return self.align_orientation_to_reference()

    # ============================================================
    # GUI callbacks
    # ============================================================

    def register_callback(self, callback: Callable[[Dict], None]):
        self.callbacks.append(callback)

    def notify_callbacks(self, data: Dict):
        for callback in self.callbacks:
            try:
                callback(data)
            except Exception as exc:
                self.get_logger().warn(f"GUI callback failed: {exc}")

    # ============================================================
    # Shutdown
    # ============================================================

    def shutdown(self):
        self.is_connected = False

        try:
            self.stop_simulation()
        except Exception:
            pass

        self.destroy_node()