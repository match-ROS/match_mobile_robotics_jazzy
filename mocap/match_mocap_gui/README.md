# MuR Mocap GUI

**Required: configure your workspace path in `~/.bashrc` before using these commands.**
Replace the example directory below with the actual name and location of your
workspace. `WS` must point to the workspace root containing `src/`, not to a
repository inside `src/`. There is no automatic workspace discovery.

```bash
# Add these lines to ~/.bashrc; adjust the workspace folder to match your setup.
export WS="$HOME/your_workspace_folder"
source /opt/ros/jazzy/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-62}"
```

Then reload and verify the configured path:

```bash
source ~/.bashrc
printf 'Workspace: %s\n' "$WS"
test -d "$WS/src" && echo "Workspace src directory found" || echo "Fix WS in ~/.bashrc"
```

Check that the printed path matches your actual workspace folder name. Update
`WS` whenever you move or rename the workspace. Remote workspaces can differ;
set `REMOTE_WS` or the GUI's Remote WS field to the path on the remote computer.


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
and the tracked QTM rigid-body frames coincide with each robot's `base_link`.

When map output is enabled (default), the GUI starts the bridge with Qualisys
robot TF enabled. The robot-model frame `<mur>/base_footprint` follows the raw,
full-6D `/qualisys_map/<mur>/pose` at up to 100 Hz. The URDF's identity joint
from `base_footprint` to `base_link` gives the exact Qualisys `base_link` pose
without a second TF parent. Set RViz Fixed Frame to `map`; the smoothed pose
remains available as a separate topic but does not drive TF.

The Cooperative Handling GUI now has a **Mocap** tab with the same local bridge
start/stop behavior plus **Freeze selected MuRs** and **Resume selected MuRs**.
These controls call the bridge's per-robot `std_srvs/SetBool` service
`/qualisys/<robot>/freeze_localization`. Holding localization keeps the last
fresh map pose and robot TF alive through temporary QTM occlusion; raw QTM
poses continue for diagnosis. The standalone Mocap GUI remains a monitor and
bridge launcher.

## Commissioning: install the Ubuntu desktop shortcut

As part of initial setup on each Ubuntu operator PC, after building this package
and `match_mur_gui` in the Jazzy workspace, install its
application-search entry and matching desktop icon with:

```bash
python3 "$WS"/src/match_mur_gui/scripts/install_gui_desktop.py --workspace "$WS" --app mocap
```

The shared installer in `match_mur_gui` checks the executable and creates a
user-level launcher that sources Jazzy and the workspace before opening the GUI.
Use `--workspace PATH` for a different Colcon workspace. No terminal is needed
to launch “MuR Mocap” afterward.

The shortcut appears on the desktop and in the application menu for the current
user, without `sudo`. If required, right-click it and select **Allow Launching**
(**Starten erlauben**). Re-run the installer after moving the workspace.
For the Basis, Mocap and Navigation shortcuts together, follow the
[repository setup step](../../README.md#gui-desktop-shortcuts-commissioning-on-the-operator-pc).
