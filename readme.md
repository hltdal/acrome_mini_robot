the codes that you will run to launch program

1. Terminal

colcon build --packages-select set_slam
source install/setup.bash
ros2 launch set_slam display.launch.py

2. Lidar verisini wsl de /scan topic i altında yayınlatma

    2.1. wsl de -> cd ~/acrome_mini_robot && ros2 run publish_lidar get_scan_from_raspbery
    (sudo fuser -k 5007/tcp) bu kod gerekebilir

    2.2. windowsta -> lidar_tools.py adlı dosya başlatılacak

    2.3. raspbery -> cd /home/halit/set_slam_ws && ros2 run lidar_pkg get_lidar

    2.4. raspbery -> cd /home/halit/lidar_ws && ros2 launch sllidar_ros2 view_sllidar_a1_launch.py

    2.5. wsl de -> cd ~/acrome_mini_robot && ros2 run publish_lidar publish_synchronous

3. Odometry verisini wsl de /odom topic i altında yayınlatma

    1.1. wsl de -> python3 /home/halit/acrome_mini_robot/src/set_slam/set_slam/launch/teleop.py

    1.2. windowsta -> odometry_tools.py adlı dosya çalıştırılacak

    1.3. raspbery de -> cd set_slam_ws && sudo -E python3 get_rpm.py

4. Terminal
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
ros2 lifecycle get /slam_toolbox
