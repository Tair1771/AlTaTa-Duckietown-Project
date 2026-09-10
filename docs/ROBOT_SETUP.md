# Robot runtime

The project targets duck2's installed environment, recorded in `config/duck2.json`:

| Component | Value |
| --- | --- |
| Architecture / host | ARM64 / Ubuntu 18.04.6 |
| Container ROS / Python | Noetic / 3.8.10 |
| OpenCV / NumPy | 4.2.0 / 1.17.4 |
| Base | Digest-pinned, architecture-specific `duckietown/dt-ros-commons:v4.3.0` |
| Camera | `/duck2/camera_node/image/compressed`, `sensor_msgs/CompressedImage` |
| Wheels | `/duck2/wheels_driver_node/wheels_cmd`, `duckietown_msgs/WheelsCmdStamped` |
| ROS master | `http://duck2.local:11311/` |

Windows SSH is the management path; the laptop does not need direct ROS routing
through WSL. These settings describe this robot and course, not universal
calibration for other vehicles.

`tools/Start-Duck2-DrivingMode.cmd` stages current source, reads the continuous
launcher, uses the installed pinned ARM64 base and prepares `duck2-companion-driving`.
It suspends `car-interface` while retaining drivers. It verifies exclusive
ownership and zero output before releasing a prior driver stop latch. It sends
no route or Continue. Controller, gateways and watchdog must remain together.

Source changes require a freshly prepared container and reopened app.
[USAGE](USAGE.md#finish-and-recover) explains cleanup and deliberate restoration
of normal control. Detailed read-only runtime inspection is available with:

```powershell
py -3 tools/inspect_robot_setup.py --connect --full --save-summary
```

Credentials and dated connection caches remain outside Git. Cached reachability
never authorizes movement.

## Optional image build

Docker Desktop is needed for local packaging, not normal source-mounted startup.
The Dockerfile builds the ROS package and preserves runtime licensing. The
default launcher starts no application.

```powershell
py -3 tools/build_local.py
py -3 tools/build_local.py --arch arm64v8 --pull
```

The first builds AMD64; the second downloads the pinned base and builds ARM64.
Cross-architecture builds depend on Docker emulation support. Neither deploys
nor starts duck2.
