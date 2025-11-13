import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = get_package_share_directory('set_slam')

    urdf_file = os.path.join(pkg_share, 'urdf', 'acrome_mini_robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Gazebo başlat
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Robotu spawn et
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc,
            '-name', 'acrome_mini_robot',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # ROS2 Control Node (controller_manager)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc},
                    os.path.join(pkg_share, 'config', 'acrome_controller.yaml')],
        output='screen'
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': use_sim_time}],
        output='screen'
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Controller'ı başlat (load + start)
    load_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        spawn_entity,
        controller_manager,
        load_joint_state_broadcaster,
        load_velocity_controller,
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])
