cd ../../../../..
colcon build --symlink-install
source install/setup.bash
clear
#ros2 launch mur_launch_sim multi_mur620.launch.py 
# Fall B: ohne Gazebo GUI
ros2 launch mur_launch_sim multi_mur_base.launch.py gz_gui:=true launch_rviz:=true lidar_sensor_type:=lidar

