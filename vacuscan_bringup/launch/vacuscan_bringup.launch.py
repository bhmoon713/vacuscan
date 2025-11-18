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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='vacuscan_rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        rviz,
    ])
