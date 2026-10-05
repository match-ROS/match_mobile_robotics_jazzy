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

## Explicit Cartesian motion activation

The General/Cooperative GUI loads the integrated Cartesian controller **inactive**
on Start Hardware, with the other joint motion controllers inactive as well.
UR power-on, brake release and External Control program startup still follow the
existing startup/recovery sequence. Reverse-interface readiness does not imply
that Cartesian control is active.

- Cooperative `START MOTION` activates Cartesian control after the existing arm
  readiness checks, then starts virtual-object control.
- In the Jog dialog, press `Enable Cartesian control` explicitly before jogging.
  Opening the dialog alone does not activate admittance.
- An explicit Align action activates Cartesian control before starting alignment.
- MoveIt/Home uses its trajectory controller and only restores Cartesian control
  if it was already active before the goal.
- `STOP ARM MOTION` and Cooperative `STOP MOTION` also deactivate Cartesian
  control; zero twist alone does not disable force-driven admittance.

Each controller activation clears old twist references and initializes the target
from the measured pose. With FT enabled, calibration holds zero joint commands
for a complete stationary sampling window (`wrench_bias_duration`, default 1 s),
regardless of `require_wrench`. All six FT values and joint velocities must be
finite; joint speeds must remain at or below 0.01 rad/s. Movement or missing
samples restart calibration. During calibration the target tracks the measured
pose and incoming motion references are discarded. Fresh commands are needed
afterwards. Missing FT data therefore blocks initial calibration even when
`require_wrench=false`.

After synchronizing these changes, rebuild `mur_control` and `match_mur_gui` on
each robot before using the GUI (the GUI's Build before launch includes both).
This change does not alter automatic Dashboard stop recovery.

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

For persistent setup on the robot host, run once:

```bash
sudo bash ~/colcon_ws/src/match_mobile_robotics_jazzy/setup_bms_can_host.sh
```

This installs `mur-bms-can.service` and a udev rule for `can0`. The adapter is
configured at 250 kbit/s on boot and USB reconnect, and activated immediately
if present. The software provisioning stage also installs this setup. An already
active bus is left unchanged; an unexpected bitrate is reported as an error.
No ROS rebuild is needed. This setup targets the default `can0` interface only.

Check with `systemctl status mur-bms-can.service` and `ip -details link show can0`.
The GUI's `sudo -n ip link ...` fallback cannot configure the adapter when sudo
requires a password; configuring it once with `ip link` alone does not survive
a reboot or USB reconnect. `Network is down` means the local CAN interface is
administratively down; it does not mean the robot's Ethernet connection is down.

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

### TF ownership with Qualisys localization

MuR hardware bringup defaults `mir_tf_excluded_children` to `odom base_footprint base_link`.
The MiR bridge filters these child frames from both `/tf` and `/tf_static`, after adding
the robot prefix. Qualisys owns `map -> <robot>/base_footprint`; the MuR robot description
owns `<robot>/base_footprint -> <robot>/base_link`. Sensor transforms and MiR odometry
messages remain available. Publishing the MiR's `odom -> base_footprint` concurrently
with Qualisys gives the base two parents and causes large jumps between the two maps.
For native MiR localization without Qualisys robot TF, explicitly pass
`mir_tf_excluded_children:=''`. Standalone MiR launches keep their original TF behavior;
the equivalent bridge/launch parameter is `tf_excluded_children` (empty by default).
Restart the MiR bridge after changing this setting. Existing TF buffers may retain old
static transforms until their consumers restart.
