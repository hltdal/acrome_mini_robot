# acrome_mini_robot
This repository exists both to control acrome mini robot in real_life with hardwares and to be able to slam.

0. raspbery deki get_rpm.py ve get_lidar.py dosyalarındaki ip adreslerini kontrol et

1. Odometry verisini wsl de /odom topic i altında yayınlatma
1.1. wsl de -> cd raspberry && ros2 run publish_odom get_odometry_from_rasbery
1.2. windowsta -> odometry_tools.py adlı dosya çalıştırılacak
1.3.1. raspbery de -> cd set_slam_ws && sudo -E python3 get_rpm.py
1.3.2. raspbery de -> eğer motorları ilk defa çalıştıracaksan (cd set_slam_ws && sudo -E python3 first_run_motors.py)

2. Lidar verisini wsl de /scan topic i altında yayınlatma
2.1. wsl de -> ros2 run publish_lidar get_scan_from_raspbery
2.2. windowsta -> lidar_tools.py adlı dosya başlatılacak
2.3. raspbery -> cd set_slam_ws && ros2 run lidar_pkg get_lidar
2.4. raspbery -> cd lidar_ws && ros2 launch sllidar_ros2 view_sllidar_a1_launch.py

3. Map verisini /map topic i altında yayınlatma
3.1. cd raspberry && ros2 launch set_slam slam_launch.py
3.2. rviz2
3.3. ros2 lifecycle set /slam_toolbox configure && ros2 lifecycle set /slam_toolbox activate
