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


## Automatic UR startup

`mur_620.launch.py` defaults to `auto_start_urs:=true`. For each enabled real
arm, `ur_startup_enable.py` requests robot mode `RUNNING` through the driver's
`ur_robot_state_helper/set_mode` action after controller spawning. This handles
power-on and brake release, then starts the UR program.

The GUI's host preflight therefore accepts `POWER_OFF` when the dashboard is
reachable. Remote Control must be enabled on robots supporting that mode;
safety stops and faults still block startup. The host check itself only queries
the dashboard and does not switch on the arm. With `auto_start_urs:=false`,
power-on and brake release must be handled separately.

After updating `setup_mur_hardware_host.sh` locally, use the GUI's Connect with
code synchronization enabled before Start Hardware to update the remote check.

## BMS battery state

The hardware launch starts `bms_can_node.py` by default. It queries the
superstructure BMS over SocketCAN and publishes:

- `/<robot_name>/bms_status/SOC` as `std_msgs/msg/Float32` in percent, matching
  the ROS 1 topic shape
- `/<robot_name>/bms_status/battery_state` as `sensor_msgs/msg/BatteryState`

The selected `robot_profile` provides the default `battery_node_id`; override it
when needed:

```bash
ros2 launch mur_launch_hardware mur_620.launch.py \
  robot_profile:=mur620d \
  battery_node_id:=0x0440
```

Bring up the SocketCAN interface before launching, for example:

```bash
sudo ip link set can0 up type can bitrate 250000
```

Alternatively, set `bms_configure_can_interface:=true` when the node has
sufficient privileges. Disable the BMS node with `launch_bms:=false`.

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

The `ewellix_driver` package links the vendored `serial` library into a
shared library. From the workspace root, use the repository's colcon metadata
to enable position-independent code for `serial`:

```bash
colcon build --symlink-install \
  --metas src/match_mobile_robotics_jazzy/colcon.meta
```
