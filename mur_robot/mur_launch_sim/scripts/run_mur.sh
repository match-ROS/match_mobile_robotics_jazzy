cd ../../../../..
colcon build --symlink-install
source install/setup.bash
clear
#ros2 launch mur_launch_sim multi_mur620.launch.py 
# Fall B: ohne Gazebo GUI
ros2 launch mur_launch_sim multi_mur620.launch.py  launch_rviz:=true launch_moveit:=true 

