# MuR Mocap GUI

Run with `ros2 run match_mocap_gui mocap_gui` after sourcing ROS 2 Jazzy and the
workspace install. It defaults to ROS domain 62 unless `ROS_DOMAIN_ID` is
already set. The application extends `MurBaseGui` like the OAK camera GUI.
Select MuRs in the standard robot selector; the Mocap panel displays raw pose
availability and input frequency, plus smoothed x/y/yaw in the QTM `mocap`
frame. The panel updates at 5 Hz and marks a pose stale after 0.5 seconds.

**Start Mocap** starts the local Qualisys SSH bridge at 100 Hz. **Stop Mocap**
stops only the bridge started by this GUI. The ROS 2 bridge publishes raw
`geometry_msgs/PoseStamped` on `/qualisys/<mur>/pose` and smoothed planar
`geometry_msgs/PoseStamped` on `/qualisys/<mur>/pose_smoothed` at up to 10 Hz.
The 0.2-second moving average uses circular mean for yaw. The smoothed pose
has z=0 and roll=pitch=0; its timestamp is the mean publication time of the
samples in the window. This smoothing adds about 0.1 seconds of group delay
for moving robots. Neither topic is transformed into the robot's `base_link`.
