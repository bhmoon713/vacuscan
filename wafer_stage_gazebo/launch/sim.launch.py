from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = PathJoinSubstitution([
        get_package_share_directory('wafer_stage_gazebo'), 'worlds', 'empty.world'
    ])
    desc_pkg = get_package_share_directory('wafer_stage_description')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'wafer_stage.urdf.xacro')

    # IMPORTANT: 'xacro' and the file path must be separate args
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        output='screen'
    )

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'wafer_stage', '-topic', 'robot_description'],
        output='screen'
    )

    spawner_js = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen'
    )
    spawner_xy = Node(
        package='controller_manager', executable='spawner',
        arguments=['xy_position_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo, rsp, spawn, spawner_js, spawner_xy
    ])
