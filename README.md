# Duck2: lane following and offline command previews

ROS 2 migration candidate: see [implementation, isolated tests and ordered hardware
handoff](docs/ROS2_READINESS.md). The shared autonomy engine now has ROS 1 and ROS 2
adapters; ROS 2 wheel output is deliberately diagnostic-only until physical safety
and platform compatibility are verified. Existing ROS 1 deployment remains the baseline.

This is the Duck2 university project. It contains a ROS 1 lane follower,
diagnostic launchers, an interpretation-only offline chatbot and a separate
offline command-preview window. The chatbot needs no API key and neither
offline app connects to a robot.

The project is matched locally to the ROS runtime installed on duck2. A live
camera compatibility check succeeded with driving disabled and the wheel output
redirected to a diagnostic topic. The installed driver completed one
upside-down left-wheel and one right-wheel hardware check. The project
controller also completed one bounded, approximately one-second lifted-wheel
check and one short upright forward check. No persistent deployment or
autonomous track-driving test has been performed.

## What is included

| Component | What it does now | What it does not prove |
| --- | --- | --- |
| Offline interpreter | Understands a curated set of English requests, follow-ups and corrections | Live robot status or execution |
| Offline command preview | Prepares basic `stop`, speed-step and next-turn drafts and records local test copies | ROS delivery or controller acceptance |
| Lane follower | Processes compressed camera images, detects existing yellow/white lane markings and publishes wheel messages | Correct physical colour, steering or speed calibration |
| Safety logic | Validates settings, rejects bad camera timestamps, stops on lane loss and publishes zero on shutdown | Physical braking distance |
| Duck avoidance prototype | Optional, disabled-by-default candidate detection and passing state | Reliable obstacle avoidance on the course |
| Runtime packaging | Builds the package over duck2's pinned ROS Noetic base image | A deployed or running controller |

The confirmed ROS interfaces are:

| Purpose | Interface | Type |
| --- | --- | --- |
| Camera input | `/${VEHICLE_NAME}/camera_node/image/compressed` | `sensor_msgs/CompressedImage` |
| Wheel output | `/${VEHICLE_NAME}/wheels_driver_node/wheels_cmd` | `duckietown_msgs/WheelsCmdStamped` |
| High-level request | `/${VEHICLE_NAME}/lane_follower/command` | JSON in `std_msgs/String` |
| Status | `/${VEHICLE_NAME}/lane_follower/status` | JSON in `std_msgs/String` |

`VEHICLE_NAME` is supplied by Duckietown at runtime; our robot name is `duck2`.

## Get the project and keep one working copy

Install Git or GitHub Desktop. In PowerShell, choose the parent folder where
you want to work, then run:

```powershell
git clone https://github.com/Tair1771/AlTaTa-Duckietown-Project.git
cd AlTaTa-Duckietown-Project
git branch --show-current
```

The shared branch is `main`. In GitHub Desktop, **File → Clone repository → URL**
does the same job. If the original checkout already exists, add that folder to
GitHub Desktop instead of creating another upload copy. This project's existing
WSL checkout is `/home/tkezdekbayev/duckietown/duckiebot-ros`.

Before starting shared work, commit your own changes and fetch/pull `main`.
With a clean checkout, `git pull --ff-only origin main` updates without making
an automatic merge. If it reports divergence, resolve it before continuing;
do not force-push or discard another teammate's work.

## Windows setup: offline apps

These steps are sufficient for the chatbot and command-preview work. Docker,
ROS, a bot and an API key are not required.

1. Clone this repository or use **Code → Download ZIP** on GitHub, then open
   Windows PowerShell in the repository folder.
2. Install current Windows Python from [python.org](https://www.python.org/downloads/windows/), including Tcl/Tk and the Python launcher.
3. Check Python and Tk:

   ```powershell
   py -3 --version
   py -3 -m tkinter
   ```

   Close the small Tk test window.

   If `py` is unavailable but `python --version` works, substitute `python`
   for `py -3`. If neither works, install Python and reopen PowerShell. No pip
   dependencies are needed for the two offline apps. The recorded Windows
   desktop tests used Python 3.12. Codex and its bundled runtime are optional.
4. Start the command-preview window:

   ```powershell
   py -3 laptop/command_preview_chat.py
   ```

   Or double-click `laptop/Start-CommandPreview.cmd`. Its title explicitly says
   **Offline command preview — no robot connection**.
5. For interpretation only, run:

   ```powershell
   py -3 laptop/offline_chat.py
   ```

   Or double-click `laptop/Start-OfflineChat.cmd`.

Try `Take the next right`, then `Actually, left`. **Record preview** stores a
local test record only. `Slow down a little` asks whether the controller's
standard fixed speed step should be used; it never silently converts “a little”
into a measured speed. Timed/distance stops, pauses, reverse, undo and
interrupt-with-cancellation are understood where possible but remain unavailable
for execution.

Run the Windows chatbot checks with:

```powershell
py -3 -X utf8 tests/test_offline_interpreter.py
py -3 -X utf8 tests/test_offline_chat_ui.py
py -3 -X utf8 tests/test_command_preview.py
py -3 -X utf8 tests/test_chat.py
```

## Local ROS package build

The build uses the exact Duckietown ROS base release observed on duck2, rather
than the old template's branch-derived Daffy build. Install Docker Desktop with
Linux containers. From a Windows checkout, build a local laptop-check image:

```powershell
py -3 tools/build_local.py
```

To check that the package also builds against the matching ARM64 base:

```powershell
py -3 tools/build_local.py --arch arm64v8 --pull
```

The script permits only a local Docker endpoint and builds with networking
disabled. It does not connect to duck2 or deploy an image. For a WSL checkout,
use the same command from Windows PowerShell through its
`\\wsl.localhost\\Ubuntu-22.04\\home\\...` path, or first enable Docker Desktop's
WSL integration.

The default image command starts no application node. For exact inspected
runtime facts, package details and the deferred live stage, read
[docs/ROBOT_SETUP.md](docs/ROBOT_SETUP.md). To run isolated regressions, use
[docs/TESTING.md](docs/TESTING.md).

### Docker and WSL prerequisites

On Windows, install Docker Desktop, select Linux containers, and start it.
Use its local `desktop-linux` context; do not select a robot Docker endpoint.
Check the client and server before building:

```powershell
docker context use desktop-linux
docker version
```

The initial image download requires internet. Build steps run with networking
disabled; an ARM64 build on an Intel/AMD laptop uses Docker's emulation and
can take longer. The image tags produced are `altata-duck2:noetic-amd64` and
`altata-duck2:noetic-arm64v8`. Rebuild after editing package source or launchers.
These commands do not install ROS on Windows or replace the robot's runtime.

WSL is optional for the offline apps and Windows Docker workflow. To create an
Ubuntu environment, run `wsl --install -d Ubuntu-22.04` in administrator
PowerShell, restart if requested, and create a Linux user. Enable Docker
Desktop's **Settings → Resources → WSL Integration → Ubuntu-22.04** if using
Docker inside Ubuntu. `wsl --list --verbose` should show WSL version 2.

For the existing WSL checkout, Windows PowerShell can run the build directly:

```powershell
py -3 "\\wsl.localhost\Ubuntu-22.04\home\tkezdekbayev\duckietown\duckiebot-ros\tools\build_local.py"
```

Teammates must substitute their own Linux username/path. Do not reinstall or
clone over an existing checkout. The previous `dts devel build` template
workflow is no longer the project's build procedure.

### Run the ROS checks from Windows PowerShell

Set `$repoPath` to the repository's absolute path. For a Windows checkout,
`(Get-Location).Path` works; for the existing WSL checkout use the UNC path
shown above, ending at `duckiebot-ros`.

```powershell
$repoPath = (Get-Location).Path
docker run --rm --network none --mount "type=bind,source=$repoPath,target=/project,readonly" --entrypoint bash altata-duck2:noetic-amd64 -lc 'source /environment.sh >/dev/null 2>&1; cd /project/tests && python3 -m unittest test_runtime_setup test_lane_follower test_navigation test_obstacles_and_connection test_first_test_readiness test_bounded_ground_supervisor test_duck_avoidance test_chat test_offline_interpreter test_command_preview test_obstacle_language'
docker run --rm --network none --mount "type=bind,source=$repoPath,target=/project,readonly" --entrypoint bash altata-duck2:noetic-amd64 -lc 'source /environment.sh >/dev/null 2>&1; python3 /project/tests/test_ros_transport.py'
```

Require an exit code of zero (`$LASTEXITCODE` in PowerShell). The second check
starts a private ROS master and synthetic camera inside the isolated container;
its wheel messages cannot reach duck2. A missing `/project/tests` directory
means the mount points to the wrong folder. A Docker connection error means
Docker Desktop or the selected local context needs checking.

## Connect to duck2 for a future live session

Use Windows PowerShell for duck2 management. The previous WSL hostname route
was not reliable. Power the robot and laptop hotspot, then allow 2–5 minutes
for boot.

Once, while duck2 is reachable, run this from Windows PowerShell in the
checkout:

```powershell
powershell -ExecutionPolicy Bypass -File tools\setup_duck2_ssh.ps1
```

The script creates a protected key at `%USERPROFILE%\.ssh\duck2_ed25519`,
asks you to enter its passphrase privately, uses the existing robot password
once to install only the public key, and verifies key-based login. It never
stores a password or IP address in the repository. If Windows asks for
administrator access to start the OpenSSH Authentication Agent, start that
service in an elevated PowerShell, rerun the script, and unlock the key using:

```powershell
ssh-add $env:USERPROFILE\.ssh\duck2_ed25519
```

Afterward, double-click `tools\Start-Duck2-Check.cmd`, or run:

```powershell
py -3 tools\inspect_robot_setup.py --connect --save-summary
```

This is the short, read-only check. It resolves duck2, verifies the saved host
identity and key login, checks essential containers and ROS master availability,
and reports the normal wheel publisher. It does not subscribe to images,
publish messages or call motor services. The non-secret dated summary is saved
under `%LOCALAPPDATA%\Duck2\connection-checks`, outside the repository.
Use `--full` only after a runtime change or when a full compatibility record
is needed.

If the robot address changes, the alias still resolves through `duck2.local`.
An unfamiliar host key is a stop condition; do not bypass it. A locked-key,
name-resolution, network or login failure gives one targeted next step and
does not retry automatically.

Windows SSH was verified on 2026-09-08. WSL hostname resolution failed in that
session; Docker-to-robot connectivity and a general laptop ROS deployment path
are not established. The successful live diagnostic ran the project's source
temporarily inside the robot's existing ROS container, with driving disabled
and output directed to a diagnostic topic. There is no persistent deployment.

The node accepts optional `~camera_topic` and `~wheels_topic` parameters;
defaults are the camera and real wheel interfaces listed above. Using a
diagnostic wheel topic permits stationary perception checks. The ordinary
`lane-camera-debug` launcher redirects output to
`/duck2/lane_follower/diagnostic_wheels_cmd`, disables driving and disables the
optional obstacle/avoidance functions.

## Verified progress and work remaining

On 2026-09-08 both architecture builds and a fresh reconstructed local-copy
build succeeded. The final unit run contained **132 tests: 131 passed and one
platform-specific test was skipped**. The isolated ROS transport suite passed,
including camera freshness, shutdown, heartbeat, gateway, recorder and route
checks. Native Windows GUI checks are a separate command listed above.

Twelve live camera frames decoded at 640×480 with advancing timestamps; the
latest observed frame age was 0.028 seconds. The application processed live
frames during a bounded diagnostic run and reported zero output. This is
software compatibility evidence, not successful lane following on a track.

The installed left/right driver checks and a bounded project-controller
lifted-wheel check succeeded. Subsequent supervised ground checks confirmed
positive forward motion, prompt stopping and roughly straight travel from a
centred start at the test-only `0.05` wheel-speed cap. The checked supervisor
has since performed several short and long ground windows, restoring normal
control after each run. It is a temporary diagnostic tool, not a deployment
launcher or evidence of reliable autonomous driving.

Track trials showed that the existing camera settings can temporarily lose the
yellow divider and enter the white-only fallback on a left curve. The follower
continued to request left steering, but the robot later approached the white
border. The installed motor driver maps small normalized commands to similar
minimum PWM values, so the requested left/right difference may not produce a
large physical turn. Steering calibration, curve reliability, red-line
behaviour, obstacle avoidance, routes and reverse remain unverified.
Automatic passing remains disabled and chatbot previews remain offline.

## Important current limits

- `/duck2/kinematics_node` normally publishes on the robot's wheel topic. The
  lifted test used a verified reversible handover by pausing `car-interface`
  and restoring it after explicit zero and emergency-stop commands.
- Do not run a ground-driving launcher until duck2 is upright on the track, a
  person is beside it, and the ground-test preflight is deliberately started.
- The verified live-management path is Windows OpenSSH using the protected
  duck2 key. The charging cable may remain connected for tests when its slack
  is clear of the wheels and track.
- Existing colour defaults detected both boundaries on straights but entered a
  white-only fallback during a curve. Colour calibration, curve steering,
  red-line driving, obstacle passing and route turns still require physical
  measurement.
- The offline chatbot and preview window have no network, ROS or robot-delivery
  path.

## Repository layout

```text
config/duck2.json                         Verified non-secret runtime facts
tools/build_local.py                      Local, pinned-image package build
tools/inspect_robot_setup.py              Opt-in read-only setup inspector
packages/duckie_lane_follower/src/        Lane follower, gateway and recorder
launchers/                                Default-safe and future live presets
laptop/                                   Offline apps and connected companion
tests/                                    Offline, synthetic and isolated ROS tests
docs/                                     Setup, safety, interface and test notes
```

Keep credentials, `.env` files, captures and generated build folders out of
Git. Docker images, Python/WSL installations, chat history and the physical map
are not included by a normal clone.

## Attribution

The initial repository was created from Duckietown's ROS template. Its retained
licence material remains in `LICENSE.pdf`. Current package code and project
documentation were developed for this team; other teams' code, motion logic and
colour values were not copied. The external runtime interfaces used here are
documented in [docs/INTERFACE_REFERENCES.md](docs/INTERFACE_REFERENCES.md).
