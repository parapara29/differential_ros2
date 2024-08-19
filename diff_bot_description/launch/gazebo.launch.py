from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    share_dir = get_package_share_directory('diff_bot_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'diff_bot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf, 'use_sim_time':use_sim_time}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )
    
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'diff_bot',
            '-topic', 'robot_description',
            '-z','1.0'
        ],
        output='screen'
    )

    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=['diff_controller']
    # )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=['joint_broad']  
    # )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_broad'],
        output='screen'
    )

    load_diff_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_controller'],
        output='screen'
    )

    return LaunchDescription([
         RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=urdf_spawn_node,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_diff_drive_base_controller],
            )
        ),
        gazebo,
        robot_state_publisher_node,
        urdf_spawn_node,
        
    ])
