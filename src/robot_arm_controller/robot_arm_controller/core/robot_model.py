import math
from dataclasses import dataclass, field
from typing import List


@dataclass
class JointModel:
    name: str
    xyz: List[float]
    rpy: List[float]
    axis: List[float]
    limit: List[float]


@dataclass
class RobotModel:
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