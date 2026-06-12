# mur_launch_hardware

Hardware bringup for the MUR platform.

## Robot profiles

The hardware launch separates the ROS namespace from the physical robot
calibration:

- `robot_name` controls the ROS namespace, for example `/mur620/...`.
- `robot_profile` selects the physical robot geometry and calibration, for
  example `mur620d`.

The default profile is `mur620d`, matching the current hardware setup. Profiles
live in `config/mur_robot_profiles.yaml` and contain:

- left/right UR kinematics calibration files
- left/right UR mounting offsets relative to the MUR top module
- whether the robot has lift columns

Example:

```bash
ros2 launch mur_launch_hardware mur_620.launch.py \
  robot_name:=mur620 \
  robot_profile:=mur620d
```

Explicit launch arguments still override the profile values:

```bash
ros2 launch mur_launch_hardware mur_620.launch.py \
  robot_profile:=mur620d \
  kinematics_params_file_r:=/path/to/calibration.yaml \
  ur_r_rpy:="0.0 0.0 2.9"
```

## Ewellix lift columns

The MUR620 launch starts one `ewellix_driver` node per lift column:

- `/<robot_name>/ewellix_lift_l/ewellix_node`
- `/<robot_name>/ewellix_lift_r/ewellix_node`

Each driver publishes `ewellix_interfaces/msg/State`. The local
`ewellix_state_to_joint_state.py` bridge converts those tick values to
`sensor_msgs/msg/JointState` for:

- `left_lift_joint`
- `right_lift_joint`

Default hardware ports are:

- left: `/dev/ttyUSB0`
- right: `/dev/ttyUSB1`

Override them at launch time:

```bash
ros2 launch mur_launch_hardware mur_620.launch.py \
  lift_port_l:=/dev/ttyUSB0 \
  lift_port_r:=/dev/ttyUSB1
```

Disable the lift drivers for dry launch checks:

```bash
ros2 launch mur_launch_hardware mur_620.launch.py \
  launch_lift_l:=false \
  launch_lift_r:=false
```

## Build note

The `ewellix_driver` package links the `serial` dependency into a shared
library. Build the workspace with position-independent code enabled so the
vendored `serial` package can be linked correctly:

```bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON
```
