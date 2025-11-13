import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Launch argument: sim zamanı kullanılsın mı?
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Paket ve dosya yolları
    pkg_share = get_package_share_directory('set_slam')
    xacro_file = PathJoinSubstitution([FindPackageShare('set_slam'), 'urdf', 'acrome_mini_robot.xacro'])
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_sync.yaml')

    # Xacro'dan URDF üret (Command() ile)
    robot_description_content = Command([
        'xacro ',
        xacro_file,
        ' ',
        'mesh_path:=', os.path.join(pkg_share, 'meshes')
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # joint_state_publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Gazebo Harmonic (ros_gz_sim) başlat
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': 'empty.sdf'}.items()
    )

    # Robotu spawn et
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'acrome_mini_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.1'  # 2 cm yukarıda spawn
        ],
        output='screen'
    )

    # SLAM toolbox (online sync)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )

    # Odom -> base_link broadcaster
    odom_to_tf_broadcaster_node = Node(
        package='set_slam',
        executable='odom_to_tf_broadcaster',
        name='odom_to_tf_broadcaster',
        output='screen'
    )

    # base_link -> laser_frame static transform publisher
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        output='screen',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame']
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Launch description oluştur
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        

        gz_sim_launch,
        spawn_robot,
        joint_state_publisher_node,
        robot_state_publisher_node,
        odom_to_tf_broadcaster_node,
        static_tf_node,
        slam_toolbox_node,
        rviz_node
    ])
