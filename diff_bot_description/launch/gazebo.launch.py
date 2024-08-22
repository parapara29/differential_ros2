from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('diff_bot_description')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    xacro_file = os.path.join(share_dir, 'urdf', 'diff_bot.xacro')
    robot_localization_file_path = os.path.join(share_dir, 'config', 'ekf.yaml')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    rviz_config_dir = os.path.join(share_dir, 'config', 'display.rviz')
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': use_sim_time},
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'true',
            'world': world,
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_controller', '--controller-manager', '/controller_manager'],
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        arguments=['joint_broad', '--controller-manager', '/controller_manager'],
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'diff_bot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[
            {'use_sim_time': use_sim_time},
            
        ],
    )

    odom_node = Node(
        package='diff_bot_description',
        executable='odom_transform',
        name = 'odom_transformer'
    )

    robot_localization = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path, 
    {'use_sim_time': use_sim_time}])

    return LaunchDescription([
        joint_broad_spawner, #The node managed by controller_manager
        diff_drive_spawner, # Node for controlling the robot
        # joint_state_publisher_node, # Commented out because we already have joint data by the joint_broad_spawner
        gazebo_server, #gazebo node
        gazebo_client, #gazebo node
        robot_state_publisher_node, #Transformation of robot links node
        urdf_spawn_node, 
        odom_node, #Custom odom TF node
        rviz_node, #RVIZ
        # robot_localization # commented cause we have custom odom running
    ])