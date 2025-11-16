#!/usr/bin/python3
# -*- coding: utf-8 -*-
from launch_ros.actions import Node
from launch import LaunchDescription


# this is the function launch  system will look for


def generate_launch_description():


    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_controller_traj_x = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["x_joint_trajectory_controller"],
        output="screen",
    )
    
    spawn_controller_traj_y = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["y_joint_trajectory_controller"],
        output="screen",
    )
    # create and return launch description object
    return LaunchDescription(
        [
            spawn_controller,
            spawn_controller_traj_x,
            spawn_controller_traj_y,
        ]
    )