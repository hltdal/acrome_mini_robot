from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # slam_toolbox param dosyası
    pkg_share = get_package_share_directory('set_slam')  # kendi paket adı
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_sync.yaml')

    return LaunchDescription([
        # 1️⃣ slam_toolbox online mapper
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',  # online_sync veya sync
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file]
        ),

        # 2️⃣ odom->base_link broadcaster
        Node(
            package='set_slam',
            executable='odom_to_tf_broadcaster',
            name='odom_to_tf_broadcaster',
            output='screen'
        ),

        # 3️⃣ base_link->laser_frame static transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            output='screen',
            arguments=['0.0','0.0','0.1','0','0','0','base_link','laser_frame']
        )
    ])
