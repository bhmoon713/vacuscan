#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # -------------------------------------------------------------
    # Paths
    # -------------------------------------------------------------
    pkg_desc = get_package_share_directory("vacuscan_description")
    pkg_gazebo = get_package_share_directory("vacuscan_gazebo")

    urdf_path = os.path.join(pkg_desc, "urdf", "vacuscan.urdf")
    rviz_config = os.path.join(pkg_desc, "rviz", "simple.rviz")
    world_path = os.path.join(pkg_gazebo, "worlds", "vacuscan_empty.world")

    # -------------------------------------------------------------
    # 1) Gazebo Classic (server + client)
    # -------------------------------------------------------------
    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            world_path,
            "-s", "libgazebo_ros_factory.so",
            "-s", "libgazebo_ros_init.so",
        ],
        output="screen"
    )

    # -------------------------------------------------------------
    # 2) Robot State Publisher (URDF → /robot_description)
    # -------------------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="vacuscan_bot_state_publisher_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_description": Command(["xacro ", urdf_path])
        }],
    )

    # -------------------------------------------------------------
    # 3) Spawn robot entity in Gazebo
    # -------------------------------------------------------------
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "vacuscan_bot",
            "-topic", "robot_description"
        ],
        output="screen",
    )

    # -------------------------------------------------------------
    # 4) ros2_control controllers
    # -------------------------------------------------------------
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    x_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["x_joint_trajectory_controller"],
        output="screen",
    )

    y_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["y_joint_trajectory_controller"],
        output="screen",
    )

    # -------------------------------------------------------------
    # 5) RViz2 (optional)
    # -------------------------------------------------------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    # -------------------------------------------------------------
    # Final combined launch
    # -------------------------------------------------------------
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        x_controller,
        y_controller,
        # rviz,   # ← enable if you want RViz
    ])
