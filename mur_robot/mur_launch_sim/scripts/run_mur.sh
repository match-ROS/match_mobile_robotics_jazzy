cd ../../../../..
colcon build --symlink-install
source install/setup.bash
clear
#ros2 launch mur_launch_sim multi_mur620.launch.py 
# Fall B: ohne Gazebo GUI
ros2 launch mur_launch_sim multi_mur620.launch.py \
  launch_moveit:=true \
  launch_rviz:=true \
  moveit_robot:=all \
  fake_localization:=true

