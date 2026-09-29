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

```bash
source /opt/ros/jazzy/setup.bash
source /home/rosmatch/colcon_ws/install/setup.bash
export ROS_DOMAIN_ID=62
ros2 run match_mocap_ros2 qualisys_ssh_bridge
```

Optional ROS parameters: `ssh_host`, `qtm_host`, `qtm_port`, `frequency`,
`frame_id`. The ROS 2 node checks its incoming stream every 5 ms. Pose timestamps
currently reflect publication time on the ROS 2 host, not the instant of
camera exposure. This is suitable for stationary calibration; synchronise
clocks and establish the capture-time offset before combining moving-camera
and mocap measurements.
The measured SSH transport delay does not include QTM camera exposure and
6D reconstruction; QTM provides a separate real-time latency indicator.
Restart the bridge after loading a different QTM rigid-body configuration so
its names are read again.

The bridge publishes the raw QTM `mocap` frame. It does not publish `/tf`,
apply the ROS 1 `map -> mocap` offset, or assume that a QTM rigid-body origin
coincides with a MuR `base_link`. Those transforms must be established for
robot-to-robot calibration.
