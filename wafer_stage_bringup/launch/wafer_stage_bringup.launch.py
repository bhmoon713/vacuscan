from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('wafer_stage_description'),
        'urdf', 'wafer_stage.urdf.xacro'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
            }],
            output='screen'
        ),
        Node(
            package='wafer_stage_control',
            executable='wafer_controller',
            name='wafer_controller',
            output='screen'
        ),
        Node(
            package='wafer_stage_control',
            executable='wafer_wire_visualizer',
            name='wafer_wire_visualizer',
            output='screen'
        ),

        Node(
            package='wafer_stage_control',
            executable='wafer_joint_broadcaster',
            name='wafer_joint_broadcaster',
            output='screen'
        ),
        Node(
            package='wafer_stage_control',
            executable='wire_length_publisher',
            name='wire_length_publisher',
            output='screen'
        ),
    ])
