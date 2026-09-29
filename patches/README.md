# Ewellix hardware-state freshness

`ewellix-communication-diagnostics-eb41860.patch` adds an acquisition-health
signal to the standalone Ewellix lift node. It applies only to the checked-in
`ewellix/ewellix_lift` revision
`eb41860fbdaefa5fa934551e54e65d384b81d59b`. The patch is kept here because the
submodule points at an external repository.

The original node continues publishing cached `state` messages when serial
communication fails. Receipt of a recent `state` or bridged `joint_states`
message therefore does **not** establish that the lift hardware responded.

## Explicit installation

From the `match_mobile_robotics_jazzy` directory:

```bash
./apply_ewellix_diagnostics_patch.sh --check
./apply_ewellix_diagnostics_patch.sh --apply
```

The default is `--check`, which makes no source changes. `--apply` is idempotent,
rejects a different revision or conflicting local changes, and preserves
unrelated work. Already-applied patches are recognized with a reverse dry run.
Use `--source-dir /path/to/ewellix_lift` for an isolated checkout. The installer
does not build or launch anything; diagnostic collection never applies patches.

Rebuild the patched driver separately, with the ROS environment sourced:

```bash
cd /home/rosmatch/colcon_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select ewellix_driver \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON
colcon test --packages-select ewellix_driver --event-handlers console_direct+
colcon test-result --verbose
```

No driver restart or robot connection is required for the tests. Existing
processes acquire this signal only after a separately scheduled restart using
the rebuilt driver. A submodule reset/update can remove the applied change;
rerun `--check` after workspace setup and apply explicitly if necessary.

## Diagnostic interface

Each node publishes `diagnostic_msgs/msg/DiagnosticArray` on its relative
`diagnostics` topic, for example
`/mur620/ewellix_lift_l/diagnostics` and
`/mur620/ewellix_lift_r/diagnostics`. The status name is
`<fully-qualified-node-name>/communication`; `hardware_id` is the configured
serial port. QoS is reliable, volatile, depth 1.

The existing publication timer emits state and diagnostics from the same
mutex-protected snapshot, at `frequency` (normally 10 Hz). Serial I/O does not
hold that mutex. A successful, decoded 98-byte Cycle2 response refreshes the
sample; cached republication cannot refresh it. The initial successful Cycle2
counts as the first sample. The decoder now reads the five error-history
records present in those 98 bytes, instead of reading past the payload.

| Key | Meaning |
| --- | --- |
| `hardware_comm_ok` | `true` only while the latest acquisition succeeded and its sample is fresh. |
| `has_sample` | Whether a valid hardware sample has ever been acquired. |
| `last_cycle_ok` | Result of the last completed Cycle2 acquisition, independent of age. |
| `last_success_age_sec` | Age measured with the driver's steady clock; `-1` before any sample. |
| `successful_cycles` | Validated sample count since node startup. |
| `consecutive_failures` | Failed acquisitions since the last valid sample. |
| `failure_reason` | Empty after success; otherwise `cycle2_failed` or `invalid_cycle2_payload_size`. |
| `stale_after_sec` | Configured freshness limit. |
| `port` | Configured serial device. |

`communication_stale_after_sec` is a positive finite node parameter, default
`2.0`. No sample or age greater than this limit produces `STALE`; a failed
latest acquisition with a still-recent previous sample produces `ERROR`;
otherwise the communication status is `OK`. ROS time is used only for the
message header, so ROS clock changes do not refresh stale hardware data.

Consumers must additionally expire the diagnostic heartbeat using their own
receive clock. Startup can fail before publishers exist, and a terminated
driver cannot report its own failure. Missing diagnostics means unknown or
unavailable hardware freshness; it is never evidence of success. `OK` here
describes communication only: it does not clear mechanical faults, certify
motion readiness, or reinterpret the lift's error history.

All existing command handling, startup activation and motion recovery remain
unchanged. The patch does not add serial requests or control commands.

## Tests

The patch includes C++ tests for exact-length decoding and five error records,
malformed payload rejection without changing the last valid sample, initial
unknown state, failure/recovery, stalled acquisition despite republication, and
coherent snapshots under concurrent access. They can be built with
AddressSanitizer/UndefinedBehaviorSanitizer without running the hardware node.

Installer checks run in temporary local clones:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s tests -p test_ewellix_diagnostics_patch.py -v
```

These verify the read-only check, repeated application, pinned revision,
preservation of changed target files, untracked collisions and unrelated work.
