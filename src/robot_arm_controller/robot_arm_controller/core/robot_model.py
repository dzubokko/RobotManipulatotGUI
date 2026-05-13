import math
from dataclasses import dataclass, field
from typing import List


@dataclass
class JointModel:
    """Description of one revolute joint in the kinematic chain."""

    name: str
    xyz: List[float]
    rpy: List[float]
    axis: List[float]
    limit: List[float]


@dataclass
class RobotModel:
    """
    Single source of truth for the manipulator configuration.

    All ROS topics, joint names, joint limits, home position and gripper
    limits should be taken from this class instead of being duplicated
    in GUI or controller files.
    """

    arm_joint_names: List[str] = field(
        default_factory=lambda: [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]
    )

    gripper_joint_names: List[str] = field(
        default_factory=lambda: [
            "left_finger_joint",
            "right_finger_joint",
        ]
    )

    arm_topic: str = "/arm_controller/joint_trajectory"
    gripper_topic: str = "/gripper_controller/joint_trajectory"
    joint_states_topic: str = "/joint_states"

    base_frame: str = "base_link"
    tool_frame: str = "tool0"

    default_arm_duration: float = 1.0
    default_gripper_duration: float = 0.7
    joint_goal_tolerance: float = math.radians(0.5)
    motion_timeout_margin: float = 1.5

    home_position: List[float] = field(
        default_factory=lambda: [
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0),
        ]
    )

    gripper_closed_position: float = 0.0
    gripper_open_position: float = 0.025

    tool_xyz: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0867])
    tool_rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    kinematic_chain: List[JointModel] = field(
        default_factory=lambda: [
            JointModel(
                name="joint_1",
                xyz=[0.0, 0.0, 0.1038],
                rpy=[0.0, 0.0, math.pi],
                axis=[0.0, 0.0, 1.0],
                limit=[-3.14, 3.14],
            ),
            JointModel(
                name="joint_2",
                xyz=[-0.04375, -0.0558, 0.08],
                rpy=[math.pi / 2.0, 0.0, 0.0],
                axis=[0.0, 0.0, 1.0],
                limit=[-0.7854, 1.5708],
            ),
            JointModel(
                name="joint_3",
                xyz=[0.0, 0.22, -0.04],
                rpy=[0.0, math.pi, 0.0],
                axis=[0.0, 0.0, 1.0],
                limit=[-1.5708, 1.5708],
            ),
            JointModel(
                name="joint_4",
                xyz=[0.07, 0.075, 0.06],
                rpy=[0.0, math.pi / 2.0, 0.0],
                axis=[0.0, 0.0, 1.0],
                limit=[-math.pi, math.pi],
            ),
            JointModel(
                name="joint_5",
                xyz=[0.0, 0.0, 0.172],
                rpy=[math.pi / 2.0, 0.0, -math.pi / 2.0],
                axis=[0.0, 0.0, 1.0],
                limit=[-2.0944, 2.0944],
            ),
            JointModel(
                name="joint_6",
                xyz=[0.0, 0.08593, 0.0],
                rpy=[-math.pi / 2.0, 0.0, 0.0],
                axis=[0.0, 0.0, 1.0],
                limit=[-3.14, 3.14],
            ),
        ]
    )

    @property
    def joint_count(self) -> int:
        return len(self.arm_joint_names)

    def clamp(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, float(value)))

    def clamp_arm_joints(self, joints: List[float]) -> List[float]:
        if len(joints) != self.joint_count:
            raise ValueError(
                f"Expected {self.joint_count} arm joints, got {len(joints)}"
            )

        result: List[float] = []
        for value, joint in zip(joints, self.kinematic_chain):
            low, high = joint.limit
            result.append(self.clamp(value, low, high))
        return result

    def clamp_gripper_opening(self, opening: float) -> float:
        return self.clamp(
            opening,
            self.gripper_closed_position,
            self.gripper_open_position,
        )

    def validate(self) -> None:
        if len(self.arm_joint_names) != len(self.kinematic_chain):
            raise ValueError(
                "arm_joint_names and kinematic_chain must have the same length"
            )

        for expected_name, joint in zip(self.arm_joint_names, self.kinematic_chain):
            if expected_name != joint.name:
                raise ValueError(
                    f"Joint name mismatch: arm_joint_names has {expected_name}, "
                    f"kinematic_chain has {joint.name}"
                )

        if len(self.home_position) != self.joint_count:
            raise ValueError("home_position must contain exactly 6 values")

        if self.gripper_open_position <= self.gripper_closed_position:
            raise ValueError("gripper_open_position must be greater than closed position")