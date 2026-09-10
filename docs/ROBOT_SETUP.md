# duck2 runtime match

Current operational instructions: [STARTUP](STARTUP.md). On 2026-09-10 the
installed runtime/message fingerprints were reverified and isolated onboard
tests passed; see [BENCH_CHAT_CHECKS](BENCH_CHAT_CHECKS.md). Normal control may
remain stopped after test cleanup. Do not interpret SSH success as wheel ownership.

This document records the compatibility and readiness checks performed on
duck2 on 2026-09-08. It separates confirmed software/interface facts from
physical driving work that remains for the course track.

## Repeatable Windows test entry point

From Windows PowerShell at the repository, run:

```powershell
.\tools\Start-Duck2-GroundTest.cmd
```

Without arguments this is a read-only startup check: one Windows SSH invocation
verifies the trusted key login, robot hostname, running interface containers,
camera/wheel/kinematics node registration and sole normal wheel publisher. It
does not subscribe to camera frames, upload source, stop containers, or command
wheels. It is suitable after a laptop or robot restart. Boot delays, locked
keys, wrong identities and unexpected controllers fail before test handover.
It never regenerates keys, bypasses host-key checking or retries automatically.

The entry point selects the bundled Windows Python used successfully on this
laptop, then the standard Python launcher if available. It runs PowerShell
with a process-only execution-policy option for the local UNC script; it does
not change the system policy. Direct WSL invocation of the session runner is
rejected because WSL does not share Windows SSH authentication or routing.
The robot's hostname alias handles changing hotspot addresses while retaining
strict verification of its previously trusted identity.

Only after positioning duck2 and receiving a fresh Go for that specific run:

```powershell
.\tools\Start-Duck2-GroundTest.cmd -Label sharp-right-white-return -Duration 15 -Go
```

`-Go` authorizes physical movement. The same startup check runs before staging
current source and handing over wheel ownership. Fresh camera/encoder checks,
the independent watchdog, explicit stopping and post-stop ownership checks
still run for every movement session. Session records retain source hashes and
installed container image identities outside the repository. A startup pass
does not prove scene suitability, motor strength, or track performance.

Current bounded right-turn settings and evidence limits are documented in
[TURNING_DIAGNOSIS.md](TURNING_DIAGNOSIS.md). Future session records include
private boot identity/container start times and `motor_registers.jsonl` from a
read-only DB21J/HATv3 probe. Probe errors mean PWM evidence is unavailable; they
do not prove the motor accepted its command. Installed runtime/calibration is
preserved. The reader never initializes the HAT or writes hardware registers.

For stationary red-line calibration, place the front of duck2 at a measured
15 cm, 10 cm or 5 cm before the line and run without `-Go`:

```powershell
.\tools\Start-Duck2-GroundTest.cmd -Label red-line-10cm -InspectRedLine
```

This temporarily runs the current detector while emergency stop remains
latched, records the line's image position outside the repository, verifies
fresh zero feedback, and restores normal ownership. It performs no physical
movement. Use the observed line-bottom fractions to choose a provisional
5–10 cm trigger before the first moving intersection test.

After that calibration and a fresh `Go`, the dedicated one-junction form is:

```powershell
.\tools\Start-Duck2-GroundTest.cmd -Label junction-straight -Duration 15 `
  -JunctionTurn straight -RedStopTriggerBottomFraction 0.80 -Go
```

Use `left` and `right` only after straight passes. The numeric trigger above is
an example, not a calibrated value. During the authorized crossing, fresh
unmarked camera frames are expected and do not stop the profile. Normal
lane-loss stopping resumes after outgoing-lane reacquisition.

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
Both services were later run once and returned success. The user observed the
corresponding individual wheel rotations for about three seconds.

The camera was passively subscribed and twelve current frames decoded as
640x480 JPEG images with monotonically increasing timestamps. The current node
was also run from standard input inside the existing ROS container for eight
seconds with `drive_enabled:=false` and
`wheels_topic:=/duck2/lane_follower/diagnostic_wheels_cmd`. It processed real
frames, reported zero left/right values and shut down cleanly. It never
published to the real wheel topic.

No calibration was changed. The installed driver ran one bounded left-wheel and
one bounded right-wheel test while duck2 was upside down. A later bounded
project-controller test used the real wheel interface for about one second at
the stand limits, after pausing `car-interface` for exclusive ownership. Both
encoders responded, Ctrl+C and the final driver feedback were zero, and the
user saw both wheels rotate and stop. Direction was not observed. The installed
container was restored healthy; nothing was replaced or configured to start an
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

## Current live-test status

Windows OpenSSH with the protected duck2 key is the verified management path.
The hotspot/SSH setup is documented in the root README. The charging cable was
present for later ground comparisons and did not visibly change the robot's
short forward response; it may remain connected when its slack is clear of the
wheels and track.

The bounded lifted-wheel project test established a reversible control handover,
encoder response and software stopping. Later upright checks confirmed forward
motion and prompt stops at a test-only wheel cap of `0.05`; the normal
`car-interface` container was restored healthy after every run.

The installed driver applies minimum PWM values. In its source, normalized
commands of `0.03` and `0.05` map to PWM values 65 and 69 respectively, so
they can rotate the wheels at very similar power. This is relevant to curve
tests: the follower requested sustained left steering, but a small numerical
wheel difference did not guarantee a strong physical turn.

On a left curve, the current image processing sometimes used the white-only
fallback because the yellow divider was between dashes. Saved-frame replay
showed that its fixed assumed lane width moved the inferred centre to the
opposite side. The bounded camera-guided preset now opts into a focused fix:
while both boundaries are visible it measures their half-width, then reuses
that value through a short one-boundary gap. Lane loss clears the value.
Ordinary launchers retain the previous default. This still needs a supervised
curve-exit run before it can be called reliable. Colour/ROI calibration,
braking distance, red-line stopping, route timing, obstacle passing and reverse
remain unverified.

The latest eight-second straight run used the test-only smooth profile and was
accepted by the user as near-perfect for the current track placement. Its final
console summary confirmed bounded stopping, but the detailed recording was lost
before transfer and is documented honestly in `docs/TESTING.md`. On 2026-09-09
the user accepted the latest sharp-right bend as perfect for its placement; the
same recorded run stopped approximately 15–20 cm before a red line. That bend
does not establish intersection traversal. Closer stopping and straight, left
and right crossings remain separate live checks.

For subsequent authorized tests, use `tools/run_duck2_ground_test.py` from
Windows after a fresh user `Go`. It stores verified evidence under
`%LOCALAPPDATA%\Duck2\evidence`, leaves the robot copy in place, and restores
`car-interface` only after the temporary test nodes have exited. Do not manually
delete a robot evidence directory until the laptop copy contains a readable
`evidence_verified.json` and the session has been reviewed.

Earlier combined left-curve-to-straight trials exposed boundary and cable-drag
issues; later supervised runs improved after the focused fallback and cable
handling. The accepted straight and sharp-right observations remain local test
evidence, while repeatability across the full map is still unverified.
At the user's request, the camera-guided supervisor now supports explicit
15-second windows while retaining fresh-camera, ownership and stop checks.
Source staging uses one compressed SSH transfer, restoration status queries are
batched, and transfer timings are recorded locally. Use the established Windows
Python/SSH path without repeating full setup inspection for each trial.

For the isolated upright right-pivot diagnostic, keep duck2 on a clear, flat
surface, unplug the charger, and use:

```powershell
.\tools\Start-Duck2-GroundTest.cmd -Label ground-right-pivot -GroundRightPivot
```

That invocation is read-only. After the user gives a fresh `Go`, add `-Go` to
run the two-second left `0.15` / right `0.00` profile. The diagnostic retains
encoder-stall, watchdog, ownership and final-zero checks and records evidence
outside the repository.
