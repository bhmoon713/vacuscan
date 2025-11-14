from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    desc_share = get_package_share_directory('wafer_stage_description')
    bringup_share = get_package_share_directory('wafer_stage_bringup')

    xacro_file = os.path.join(desc_share, 'urdf', 'wafer_stage.urdf.xacro')
    rviz_config = os.path.join(bringup_share, 'rviz', 'wafer_stage.rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    bringup_share = get_package_share_directory('wafer_stage_bringup')
    waypoints_file = os.path.join(bringup_share, 'config', 'waypoints.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': robot_description}],
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

        # --- RViz2 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        Node(
            package='wafer_stage_control',
            executable='waypoint_runner',
            name='waypoint_runner',
            parameters=[{'waypoints_file': waypoints_file}],
            output='screen'
        ),

        Node(
            package='wafer_stage_control',
            executable='vacuum_controller',
            name='vacuum_controller',
            output='screen'
        ),
        
        Node(
            package='wafer_stage_control',
            executable='capture_controller',
            name='capture_controller',
            output='screen'
        ),

    ])
