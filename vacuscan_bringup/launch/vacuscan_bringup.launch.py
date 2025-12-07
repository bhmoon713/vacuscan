from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
import os


def generate_launch_description():
    # --- Gazebo sim from vacuscan_gazebo ---
    vacuscan_gazebo_share = get_package_share_directory('vacuscan_gazebo')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vacuscan_gazebo_share, 'launch', 'sim.launch.py')  # <- change if needed
        )
    )

    # --- RViz2 with vacuscan config ---
    bringup_share = get_package_share_directory('vacuscan_description')
    rviz_config = PathJoinSubstitution(
        [bringup_share, 'rviz', 'simple.rviz']  # we'll create a simple file
    )

    waypoint_share = get_package_share_directory('vacuscan_bringup')
    waypoints_file = os.path.join(waypoint_share, 'config', 'waypoints.yaml')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='vacuscan_rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    waypoint_runner = Node(
        package='wafer_stage_control',
        executable='waypoint_runner',
        name='waypoint_runner',
        parameters=[{'waypoints_file': waypoints_file}],
        output='screen'
    )

    vacuum_controller = Node(
        package='wafer_stage_control',
        executable='vacuum_controller',
        name='vacuum_controller',
        output='screen'
    )
        
    capture_controller = Node(
            package='wafer_stage_control',
            executable='capture_controller',
            name='capture_controller',
            output='screen'
    )

    wafer_controller = Node(
            package='wafer_stage_control',
            executable='wafer_controller',
            name='wafer_controller',
            output='screen'
    )
    wafer_joint_broadcaster = Node(
            package='wafer_stage_control',
            executable='wafer_joint_broadcaster',
            name='wafer_joint_broadcaster',
            output='screen'
    )    

    return LaunchDescription([
        gazebo_launch,
        rviz,
        waypoint_runner,
        vacuum_controller, 
        capture_controller,
        wafer_controller,
        wafer_joint_broadcaster,
    ])
