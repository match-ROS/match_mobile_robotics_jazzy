# Qualisys to ROS 2

`qualisys_ssh_bridge` runs on a ROS 2 Jazzy computer. It starts the existing
`qtm` Python SDK on `roscore` over SSH and forwards tracked 6D rigid bodies as
`geometry_msgs/PoseStamped` on `/qualisys/<body_name>/pose`. No ROS 2 install,
OS upgrade, or persistent file change on the Ubuntu 20.04 `roscore` host is
required. The ROS 1 Qualisys driver can continue to run independently.

The ROS 2 computer needs passwordless SSH access to `roscore`, and `roscore`
needs the `qtm` Python package and network access to the QTM server. The
default QTM endpoint is `QTM:22223`, the requested stream rate is 100 Hz,
and the default output frame is `mocap`. The QTM project must itself run at
100 Hz or faster to supply distinct frames at that rate.
The bridge converts QTM millimetres to metres and converts its column-major
rotation matrix to a ROS quaternion, as the existing ROS 1 driver does. Missing
or untracked bodies (NaN coordinates) are not published.

For `mur620a` through `mur620d`, the same bridge also publishes a planar
`PoseStamped` on `/qualisys/<robot>/pose_smoothed` at up to 10 Hz. It averages
x/y over the latest 0.2 s and takes a circular mean of yaw, so angles near
±180° do not cancel. The smoothed message keeps the `mocap` frame, sets z and
roll/pitch to zero, and uses the average source timestamp of its window. A
causal 0.2-second mean adds roughly 0.1 s of lag during constant-speed motion.

```bash
source /opt/ros/jazzy/setup.bash
source /home/rosmatch/colcon_ws/install/setup.bash
export ROS_DOMAIN_ID=62
ros2 run match_mocap_ros2 qualisys_ssh_bridge
```

Optional ROS parameters: `ssh_host`, `qtm_host`, `qtm_port`, `frequency`,
`frame_id`, `smoothing_window_sec`, `smoothed_rate_hz`, and
`publish_map_pose`. The ROS 2 node checks its incoming stream every 5 ms. Raw
pose timestamps reflect publication time on the ROS 2 host, not the instant of
camera exposure. Synchronise clocks and establish the capture-time offset
before combining moving-camera and mocap measurements. SSH transport delay
does not include QTM camera exposure and 6D reconstruction; QTM provides a
separate real-time latency indicator.
Restart the bridge after loading a different QTM rigid-body configuration so
its names are read again.

Map output is enabled by default. At each connection the bridge reads the
`map_to_mocap` static transform directly from
`/home/rosmatch/catkin_ws/src/match_mocap/launch_mocap/launch/mocap_launch.launch`
on `roscore`. It accepts only an unambiguous planar `map -> mocap` transform.
The currently verified numerical calibration is also pinned in
`map_transform.py` because the roscore worktree has not committed it and the
repository contains older values. If the file is missing, malformed, or
differs from the verified calibration, map output stays off and the raw stream
continues. Review and update the pin deliberately after any map recalibration. For tracked MuRs, the bridge publishes separate map-frame topics
`/qualisys_map/<robot>/pose` and `/qualisys_map/<robot>/pose_smoothed`. Set
`publish_map_pose:=false` to suppress these outputs. The original `/qualisys`
topics always remain in `mocap`. With validated map calibration, the bridge
also publishes `map -> mocap` on `/tf_static` and each tracked rigid body as
`mocap -> qualisys/<robot>` on `/tf`. The QTM rigid-body frames for A/B/C/D
coincide with the respective MuR `base_link` frames. These TF edges are
suppressed when map output is disabled or its calibration
check fails. In RViz, use Fixed Frame `map` and add a TF display to compare
these frames with the robot models.

The Mocap GUI additionally starts the bridge with `publish_robot_tf:=true` when
its map checkbox is on. Each **raw**, unsmoothed `/qualisys_map/<robot>/pose`
updates `map -> <robot>/base_footprint` at the QTM frame rate. The MuR URDF
has an identity fixed joint from `base_footprint` to `base_link`, so the
resulting `map -> <robot>/base_link` is exactly the full 6D Qualisys pose,
including z, roll, and pitch. This avoids a second parent for `base_link`.
`/qualisys_map/<robot>/pose_smoothed` is not used for TF. Verify that the
MiRs use the same map calibration before using these TF frames for motion.
The standalone bridge keeps `publish_robot_tf` disabled by default.
