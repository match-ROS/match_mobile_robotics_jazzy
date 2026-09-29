# MiR floor cameras in ROS 2

The MiR600 provides separate left and right floor camera streams through its ROS 1
rosbridge server on port 9090. The `mir_camera_bridge` node forwards raw RGB and
16-bit depth images plus their `CameraInfo` into the robot's ROS 2 namespace.
It is off by default and throttled to 2 Hz per topic.

In the MuR GUI, select **mir** and **MiR cameras (2 Hz)** before **Start Hardware**.
The camera checkbox also selects `mir`. From a launch command, use
`launch_mir:=true launch_mir_cameras:=true`; set `mir_camera_max_rate_hz:=<rate>`
when another rate is needed.

For a camera-only test without restarting the hardware, run this on the
corresponding MuR control PC after sourcing the ROS workspace:

```bash
ros2 run mir_launch_hardware mir_camera_bridge --ros-args \
  -r __ns:=/mur620c -p mir_hostname:=192.168.12.20 -p max_rate_hz:=2.0
```

For each `side` (`left` or `right`), the ROS 2 topics are:

- `/<robot>/camera_floor_<side>/driver/color/image_raw` (`sensor_msgs/Image`, `rgb8`)
- `/<robot>/camera_floor_<side>/driver/color/camera_info` (`sensor_msgs/CameraInfo`)
- `/<robot>/camera_floor_<side>/driver/depth/image_rect_raw` (`sensor_msgs/Image`, `16UC1`)
- `/<robot>/camera_floor_<side>/driver/depth/camera_info` (`sensor_msgs/CameraInfo`)

The bridge retains the MiR image timestamps and prefixes optical frame IDs with
the robot namespace. It keeps only the latest pending frame per stream and decodes
it only when the ROS 2 topic has a subscriber.

If the topics are visible on MuR620c itself but not on the GUI computer, use an
explicit ROS 2 discovery peer in that computer's shell:

```bash
export ROS_DOMAIN_ID=62
export ROS_STATIC_PEERS=mur620c
export ROS2CLI_NO_DAEMON=1
ros2 topic list | grep /mur620c/camera_floor_
```

The general MuR GUI sets `ROS_STATIC_PEERS=mur620c` by default when the variable
is otherwise unset. Restart the GUI to apply this to its ROS helper.
