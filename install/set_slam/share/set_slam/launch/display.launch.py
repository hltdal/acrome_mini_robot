import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Argümanlar
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Dosya Yolları
    pkg_share = get_package_share_directory('set_slam')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'fixed.rviz')
    xacro_file = PathJoinSubstitution([FindPackageShare('set_slam'), 'urdf', 'acrome_mini_robot.xacro'])
    ekf_config_path = PathJoinSubstitution([FindPackageShare('set_slam'), 'config', 'ekf.yaml'])
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_sync.yaml')
    world_file_path = os.path.join(pkg_share, 'worlds', 'empty_with_ground.sdf')

    # Robot Description
    robot_description_content = Command([
        'xacro ', xacro_file, ' ',
        'mesh_path:=', os.path.join(pkg_share, 'meshes')
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # 1. Robot State Publisher (TF Omurgası)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # 2. Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. Gazebo Simulation
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': ['-r ', world_file_path]}.items()
    )

    # 4. Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'acrome_mini_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.05'
        ],
        output='screen'
    )

    # 5. Bridge Node (IMU, Clock)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        output='screen'
    )

    # 6. Controllers
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('acrome_mini_robot'), 'config', 'acrome_controller.yaml',
    ])

    wheel_velocity_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['wheel_velocity_controller', '--param-file', robot_controllers],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    joint_broad_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Gazebo yerine veriyi bu Node basacak.
    real_lidar_node = Node(
        package='publish_lidar',
        executable='get_scan_from_raspbery',
        name='lidar_tcp_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}] 
    )

    # STATIC TF (lidar_link -> laser_frame)
    static_tf_lidar_correction = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar_correction',
        arguments=['0','0','0','0','0','0', 'lidar_link', 'laser_frame'],
        output='screen'
    )

    # 7. RF2O Laser Odometry (Lidar'dan odom üretir)
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/lidar_odom',
            'publish_tf': False,    # TF'i EKF basacak
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
            'use_sim_time': use_sim_time
        }]
    )

    # 8. EKF (Robot Localization) -> TF (odom->base_link) bu basacak
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
    )

    # 9. SLAM Toolbox -> TF (map->odom) bu basacak
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )

    # 10. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        
        gz_sim_launch,
        bridge,
        spawn_robot,
        robot_state_publisher_node,
        joint_state_publisher_node,
        real_lidar_node,
        static_tf_lidar_correction,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[wheel_velocity_controller_spawner, joint_broad_spawner],
            )
        ),

        # Çalışma Sırası: Sensör Verisi -> Lidar Odom -> EKF -> SLAM
        rf2o_node,
        ekf_node,
        slam_toolbox_node,
        rviz_node
    ])