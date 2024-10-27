cd ../../../..
colcon build --symlink-install
source install/setup.bash
clear
ros2 launch mur_launch_sim mur620.launch.py 