# Installed runtime and project setup

Current startup: [STARTUP](STARTUP.md). Current live operation:
[USAGE](USAGE.md). Earlier experimental commands are preserved in
[HISTORICAL_BENCH_SETUP](HISTORICAL_BENCH_SETUP.md); they are not current presets.

## Verified facts, with observation dates

The baseline was inspected on 2026-09-08. A saved full inspection on 2026-09-10
confirmed ARM64, Ubuntu 18.04.6, ROS Noetic and both message fingerprints again.
These are recorded observations, not a fresh connection check for every boot.

| Component | Recorded value |
| --- | --- |
| Host OS / architecture | Ubuntu 18.04.6 / aarch64 (arm64v8) |
| ROS runtime | Noetic in the installed ROS container |
| Container Python / OpenCV / NumPy | 3.8.10 / 4.2.0 / 1.17.4 |
| Project base | `duckietown/dt-ros-commons:v4.3.0`, architecture-specific digests in `config/duck2.json` |
| ROS master | `http://duck2.local:11311/` |
| Windows management | Protected SSH key, alias `duck2`, strict trusted host identity |
| Camera | `/duck2/camera_node/image/compressed`, `sensor_msgs/CompressedImage` |
| Wheels | `/duck2/wheels_driver_node/wheels_cmd`, `duckietown_msgs/WheelsCmdStamped` |

Camera MD5: `8f7a12909da2c9d3332d540a0977563f`.
Wheel MD5: `edbf8d24194d839b1982a6a991b552c6`.
The installed host OS and the container's Python/ROS environment are different
layers; the laptop's Ubuntu version does not determine the robot ROS version.

Windows SSH is the established path. Direct WSL/Docker-to-robot routing is not
required. No fixed hotspot address or credentials are stored in the project.

## One-time and per-session setup

Install Windows OpenSSH and Python/Tk/Pillow, establish the robot identity, and
install the protected key once using [STARTUP](STARTUP.md). Subsequent sessions
use `tools/Start-Duck2-Check.cmd`; an explicit full inspection is available via:

```powershell
py -3 tools/inspect_robot_setup.py --connect --full --save-summary
```

Use the same Windows Python chosen by the shortcuts. Summaries and cached
runtime identities remain in local application data. Repeat full inspection
when identities change or when requested; do not reinstall ROS to fix routing.

`tools/Start-Duck2-DrivingMode.cmd` stages the current Python sources using
the verified ARM64 base. It checks stopped state, suspends conflicting project
preview and normal car-interface control, establishes exclusive ownership and
starts a temporary controller awaiting a route. It prepares SSH forwarding but
does not send Start. A running app/container does not reload source edits;
Stop, close the old app and prepare again before reopening.

Normal `/duck2/kinematics_node` ownership is a baseline observation, not a
permanent fact. During project driving the project follower must own wheel
output. After cleanup normal control may remain stopped deliberately. Check
live ownership before movement rather than treating a cached result as permission.
Follow [USAGE](USAGE.md#finish-and-recover) for deliberate restoration.

## Local packaging

The Dockerfile retains the ROS package, dependencies and licence. The default
launcher starts no application. Docker Desktop builds locally; it is not
needed to prepare the source-mounted robot application.

```powershell
py -3 tools/build_local.py
py -3 tools/build_local.py --arch arm64v8 --pull
```

The builder refuses a remote Docker endpoint. These commands build AMD64 and
ARM64 compatibility images; they do not deploy or start duck2. Both builds
passed earlier in the project. No rebuild or robot deployment was performed
for the documentation-only refresh.

The installed calibration and robot startup remain unchanged. Historical
diagnostics, motor mapping and physical observations are retained in
[TESTING](TESTING.md), not promoted into universal calibration guarantees.
