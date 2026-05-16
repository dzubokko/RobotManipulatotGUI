import os
import re

import xacro

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_share = FindPackageShare("robot_description").find(
        "robot_description"
    )
    robot_gazebo_share = FindPackageShare("robot_gazebo").find("robot_gazebo")

    xacro_file = os.path.join(
        robot_description_share,
        "urdf",
        "robot.urdf.xacro",
    )

    world_file = os.path.join(
        robot_gazebo_share,
        "worlds",
        "empty.world",
    )

    robot_doc = xacro.process_file(xacro_file)
    robot_description_content = robot_doc.documentElement.toxml()

    robot_description_content = re.sub(
        r"<!--.*?-->",
        "",
        robot_description_content,
        flags=re.DOTALL,
    )

    robot_description = {
        "robot_description": robot_description_content,
    }

    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            os.path.dirname(robot_description_share),
            ":",
            os.environ.get("GAZEBO_MODEL_PATH", ""),
        ],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("gazebo_ros").find("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "world": world_file,
            "verbose": "true",
            "gui": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-entity",
                    "robot_manipulator",
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.08",
                    "-R",
                    "0.0",
                    "-P",
                    "0.0",
                    "-Y",
                    "0.0",
                ],
                output="screen",
            )
        ],
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "60",
                ],
                output="screen",
            )
        ],
    )

    arm_controller_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "60",
                ],
                output="screen",
            )
        ],
    )

    gripper_controller_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "gripper_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "60",
                ],
                output="screen",
            )
        ],
    )

    # Gazebo сначала создаёт модель с суставами около нуля.
    # Для нашей механики такая поза выглядит как "робот лежит".
    # Поэтому после загрузки контроллеров отправляем безопасную стартовую позу.
    send_start_home_pose = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/arm_controller/joint_trajectory",
                    "trajectory_msgs/msg/JointTrajectory",
                    "{joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], "
                    "points: [{positions: [0.0, 0.8, -0.8, 0.0, 0.6, 0.0], "
                    "time_from_start: {sec: 2, nanosec: 0}}]}",
                ],
                output="screen",
            )
        ],
    )

    send_start_gripper_pose = TimerAction(
        period=15.5,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/gripper_controller/joint_trajectory",
                    "trajectory_msgs/msg/JointTrajectory",
                    "{joint_names: ['left_finger_joint', 'right_finger_joint'], "
                    "points: [{positions: [0.0, 0.0], "
                    "time_from_start: {sec: 1, nanosec: 0}}]}",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            gazebo_model_path,
            gazebo_launch,
            robot_state_publisher,
            spawn_robot,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
            send_start_home_pose,
            send_start_gripper_pose,
        ]
    )