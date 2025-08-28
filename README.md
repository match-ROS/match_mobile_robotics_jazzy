<div align="center">

# match_mobile_robotics_jazzy
ROS 2 Jazzy (Ubuntu 24.04) mobile & manipulator robotics simulation + control workspace

</div>

## Overview

This repository is a ROS 2 Jazzy monorepo-style workspace containing simulation, description, control, navigation and utility packages for multiple mobile robot platforms (MiR-like, MUR, UR arm in Gazebo / gz sim) plus example four–wheel-steering control.

Key components:

* `match_gazebo`: Launches Gazebo (Ignition / gz sim) worlds (maze, empty) and bridges selected topics via `ros_gz_bridge`.
* `mir_*` packages: Description, drivers, actions, navigation, REST API, messages/services for a MiR-style platform.
* `mur_*` packages: Description, MoveIt config, simulation launch for a MUR (mobile manipulator) configuration.
* `ur_robot`: Upstream UR (Universal Robots) simulation integration (gz based) via included `.repos` manifests.
* `velocity_pub`: Example four wheel steering velocity + steering angle commander driven by joystick (`sensor_msgs/Joy`).

> NOTE: Some sub-packages have `COLCON_IGNORE` to exclude them until their dependencies are satisfied or they are ready.

## Prerequisites

Tested on Ubuntu 24.04 (Noble) with ROS 2 Jazzy.

1. Install ROS 2 Jazzy base + desktop + dev tools and dependencies (gz sim, control, MoveIt, etc.). An example script is provided in `ROS2_setup.sh` (review before executing):

```bash
chmod +x ROS2_setup.sh
./ROS2_setup.sh   # optional convenience; may prompt for sudo
```

2. Source ROS 2 environment (add to your shell RC if desired):

```bash
source /opt/ros/jazzy/setup.bash
```

3. Install additional system packages (if not already present):

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-moveit-py ros-jazzy-srdfdom joystick
sudo apt install -y ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-pcl-msgs  # required by ira_laser_tools (scan merging)
```

4. (Optional) Install `joy` package for joystick input:

```bash
sudo apt install -y ros-jazzy-joy
```

## Clone & Import External Dependencies

If you have not yet cloned this repository:

```bash
mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/match-ROS/match_mobile_robotics_jazzy.git
cd ..
```

Import any referenced external repositories (for UR simulation, etc.) using the provided `.repos` files (select the one matching Jazzy when relevant):

```bash
vcs import src < src/match_mobile_robotics_jazzy/ros2.repos
```

If `vcs` is missing:

```bash
sudo apt install -y python3-vcstool
```

## Build

From the workspace root (one level above `src/`):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Then source the local overlay:

```bash
source install/setup.bash
```

Re-build after changes:

```bash
colcon build --packages-select <pkg_name>
```

Run tests (copyright / lint / style):

```bash
colcon test --packages-select match_gazebo
colcon test-result --verbose
```

## Running Simulation (Gazebo / gz sim)

Launch gazebo with a world (default: `maze`):

```bash
ros2 launch match_gazebo gazebo.launch.py world:=maze
```

Available worlds are found in `match_gazebo/worlds/` (e.g. `empty`, `maze` – pass the base name).

The launch file internally:

* Sets `GZ_SIM_RESOURCE_PATH` to include package worlds.
* Includes `ros_gz_sim` launcher with `-v 4 -r` verbosity and real-time flag.
* Bridges topics via `ros_gz_bridge` for `/b_scan`, `/f_scan` (LaserScan) and `/clock`.

To list gz topics:

```bash
gz topic -l
```

## MUR Launch Examples

Single MUR (mur620a) using reusable base:

```bash
ros2 launch mur_launch_sim mur620.launch.py world:=maze
```

Multiple MUR robots (mur620a–d) in one world (first robot starts Gazebo, others reuse it):

```bash
ros2 launch mur_launch_sim multi_mur620.launch.py world:=maze
```

Customize spawn positions / names by cloning `multi_mur620.launch.py` or creating a new launcher that includes `mur_base.launch.py` with different `robot_name` / `x` / `y` arguments. Base arguments available:

* `robot_name` (namespace + entity name)
* `world` (only first include needs to start Gazebo with it)
* `x y z Y` (spawn pose; yaw in radians)
* `include_gz` (`true` only for first robot)
* `lidar_bridge` (bridge that robot's /scan)

Example custom include (second robot without starting gz):

```bash
ros2 launch mur_launch_sim mur_base.launch.py robot_name:=mur620b include_gz:=false x:=2.0 y:=1.0
```

## Four Wheel Steering Example (`velocity_pub`)

The `velocity_pub` package publishes steering positions and wheel velocities to controllers at:

* `/forward_position_controller/commands` (`Float64MultiArray` 4 elements)
* `/forward_velocity_controller/commands` (`Float64MultiArray` 4 elements)

Launch the node:

```bash
ros2 launch velocity_pub four_ws_control.launch.py
```

Modes (selected from joystick buttons in `robot_control.py`):

1. Opposite phase (LB)
2. In-phase (A)
3. Pivot turn (RB)
4. None (default / others)

Axes mapping (Xbox layout):

* `axes[1]` forward/back (linear x)
* `axes[0]` lateral (linear y)
* `axes[3]` yaw (angular z)

> Currently the script zeroes steering positions and sets wheel velocities to 0.5 each cycle (lines near the end of `timer_callback`) – adjust logic before deployment.

## Package Notes

* Some packages (e.g. `mir_driver`, `mir_navigation`) require additional hardware / map / config assets; ensure their dependencies are installed or selectively build using `--packages-select`.
* Ignore packages by creating a `COLCON_IGNORE` file in their directory (already used for some doc folders).
* UR simulation support: Use provided `ur_simulation_gz.jazzy.repos` if you want the full UR environment.

## Development Workflow

Formatting / lint (Python):

```bash
pip install black==24.4.2
black .
```

General ROS code style: run `ament_lint_auto` through colcon tests where configured.

Environment convenience:

Add to `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
export ROS_DOMAIN_ID=0  # or set per project
```

## Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| `package not found` | Overlay not sourced | `source install/setup.bash` |
| Missing `/clock` | Bridge not running | Ensure `match_gazebo` launch ran; check `ros2 node list` |
| Joystick no messages | `joy` not installed / permissions | Install `ros-jazzy-joy`; run `sudo chmod a+rw /dev/input/js*` |
| Build fails for UR | External repos missing | Run `vcs import` with correct `.repos` file |

## Contributing

1. Fork & create feature branch.
2. Add / update tests or simulation launch instructions if behavior changes.
3. Run formatting & lint tests.
4. Submit PR with concise description & reproduction / launch steps.

## License

See `LICENSE` file.

---

Feel free to open issues for missing instructions or feature requests.
