# duck2 runtime match

This document records the compatibility and readiness checks performed on
duck2 on 2026-09-08. It separates confirmed software/interface facts from
physical driving work that remains for the course track.

## Confirmed runtime facts

- The robot hostname is `duck2`; it was reachable from Windows over the team
  hotspot. An address observed during that session is intentionally not stored:
  hotspot addresses can change.
- Windows reached the robot through trusted SSH host keys and authenticated as
  the configured robot user. WSL could not resolve `duck2.local`, so Windows
  SSH is the verified live-management path for now. Docker Desktop is available
  locally; no Docker daemon connection to the robot is used.
- The robot host is Ubuntu 18.04 on `aarch64` (`arm64v8`). Its ROS container
  runs ROS Noetic, Python 3.8.10, OpenCV 4.2.0 and NumPy 1.17.4.
- Its ROS master is `http://duck2.local:11311/` and ROS hostname is
  `duck2.local`.
- The confirmed camera interface is
  `/${VEHICLE_NAME}/camera_node/image/compressed`
  (`sensor_msgs/CompressedImage`, MD5
  `8f7a12909da2c9d3332d540a0977563f`).
- The confirmed wheel interface is
  `/${VEHICLE_NAME}/wheels_driver_node/wheels_cmd`
  (`duckietown_msgs/WheelsCmdStamped`, MD5
  `edbf8d24194d839b1982a6a991b552c6`).

`/duck2/kinematics_node` is the existing publisher on the wheel topic. The
wheel driver subscribes to that topic and exposes the built-in left/right test
services. Their verified instructions require the Duckiebot to be upside down
with both wheels clear; each test spins one wheel for about three seconds.
Those services were inspected but not run.

The camera was passively subscribed and twelve current frames decoded as
640x480 JPEG images with monotonically increasing timestamps. The current node
was also run from standard input inside the existing ROS container for eight
seconds with `drive_enabled:=false` and
`wheels_topic:=/duck2/lane_follower/diagnostic_wheels_cmd`. It processed real
frames, reported zero left/right values and shut down cleanly. It never
published to the real wheel topic.

No calibration was changed. No real wheel command, including a zero command,
was sent. No installed container was replaced or configured to start an
application automatically.

## Local package build

`config/duck2.json` holds the non-secret facts above and pins the exact
Duckietown `dt-ros-commons:v4.3.0` image digest used by duck2. The Dockerfile
builds this project’s `duckie_lane_follower` package over that base. The default
launcher prints a message and starts no application node.

With Docker Desktop running, build an AMD64 laptop-check image from Windows
PowerShell in a normal Windows checkout:

```powershell
py -3 tools/build_local.py
```

Build the matching ARM64 image for compatibility checking:

```powershell
py -3 tools/build_local.py --arch arm64v8 --pull
```

The build tool refuses a remote Docker endpoint. It is a local packaging check;
it neither contacts duck2 nor deploys an image. If the checkout is inside WSL,
run the same command with its `\\wsl.localhost\\...` path from Windows PowerShell,
or configure Docker Desktop’s WSL integration first.

## Deferred live work

The live image shows a desk rather than the course, so its lane/obstacle result
is not calibration evidence. Colour/ROI calibration, wheel direction, motor
response, braking distance, red-line stopping, route timing and obstacle
passing remain unverified.

Before the next wheel session, establish exclusive wheel control and verify
driver stopping behaviour. The installed three-second hardware tests have not
had their speed or implementation checked and must not automatically replace
the planned one-second, low-speed pulses. Any rotation requires the user to
confirm duck2 is secured with both wheels clear and they are beside it.
Do not run this project's driving launcher until those checks pass and a safe
handover from the existing kinematics controller has been established.
