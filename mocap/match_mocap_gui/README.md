# MuR Mocap GUI

Run with `ros2 run match_mocap_gui mocap_gui` after sourcing ROS 2 Jazzy and the
workspace install. It defaults to ROS domain 62 unless `ROS_DOMAIN_ID` is
already set. The application extends `MurBaseGui` like the OAK camera GUI.
Select MuRs in the standard robot selector; the Mocap panel displays raw pose
availability and input frequency, plus smoothed x/y/yaw in the QTM `mocap`
frame. A default-on checkbox adds separate `map`-frame topics under
`/qualisys_map/<robot>/pose` and `/qualisys_map/<robot>/pose_smoothed`; its
status column confirms whether map poses are arriving. Toggling it while the
GUI-managed bridge is running restarts that bridge. The panel updates at
5 Hz and marks a pose stale after 0.5 seconds.

**Start Mocap** starts the local Qualisys SSH bridge at 100 Hz. **Stop Mocap**
stops only the bridge started by this GUI. The ROS 2 bridge publishes raw
`geometry_msgs/PoseStamped` on `/qualisys/<mur>/pose` and smoothed planar
`geometry_msgs/PoseStamped` on `/qualisys/<mur>/pose_smoothed` at up to 10 Hz.
The 0.2-second moving average uses circular mean for yaw. The smoothed pose
has z=0 and roll=pitch=0; its timestamp is the mean publication time of the
samples in the window. This smoothing adds about 0.1 seconds of group delay
for moving robots. The transform is read at bridge startup from the ROS 1
launch file on `roscore`. Raw and smoothed `/qualisys` topics stay in `mocap`,
and no topic represents the robot's `base_link` without further calibration.
