import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Bu, use_sim_time parametresini almamızı sağlar (SLAM için önemlidir)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 1. URDF dosyasının tam yolunu bul
    urdf_file_name = 'acrome_mini_robot.urdf'
    pkg_share = get_package_share_directory('set_slam')
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)

    # 2. URDF dosyasının içeriğini oku
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 3. Düğümleri Tanımla

    # Bu, hareketli joint'lerin (tekerlekler) durumunu /joint_states topiğine yayınlar
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # robot_state_publisher: URDF'i okur ve /joint_states'i dinleyerek TF yayinlar
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': use_sim_time}
        ]
    )

    # rviz2: Görselleştirme
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 4. LaunchDescription'ı oluştur ve düğümleri döndür
    return LaunchDescription([
        # Simülasyon zamanı kullanılıp kullanılmayacağını belirten argüman
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        joint_state_publisher_node,  # Bunu ekledik
        robot_state_publisher_node,
        rviz_node
    ])