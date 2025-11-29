the codes that you will run to launch program

1. Terminal

colcon build --packages-select set_slam
source install/setup.bash
ros2 launch set_slam display.launch.py

2. Terminal
python3 /home/halit/acrome_mini_robot/src/set_slam/set_slam/launch/teleop.py

3. Terminal
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
ros2 lifecycle get /slam_toolbox
