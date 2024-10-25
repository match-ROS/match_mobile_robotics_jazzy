cd ../../../..
colcon build --symlink-install
source install/setup.bash
clear
ros2 launch mur_moveit_config mur620.launch.py 