# Duck2: lane following and offline command previews

This is the Duck2 university project. It contains a ROS 1 lane follower,
diagnostic launchers, an interpretation-only offline chatbot and a separate
offline command-preview window. It also includes an offline-first combined
map, camera and chat companion. The offline chat features need no API key, and
offline planning never connects to a robot or sends a command.

The project is matched locally to the ROS runtime installed on duck2. Live
camera and wheel-interface checks succeeded, and bounded supervised track runs
have exercised straight motion, a left curve and a sharp-right bend. The latest
sharp-right run was accepted by the user and stopped on a detected red line
approximately 15–20 cm ahead. Closer red-line stopping and all straight, left
and right intersection crossings remain to be calibrated. Routes and obstacle
passing remain unverified. There is no persistent deployment.

## What is included

| Component | What it does now | What it does not prove |
| --- | --- | --- |
| Offline interpreter | Understands a curated set of English requests, follow-ups and corrections | Live robot status or execution |
| Offline command preview | Prepares basic `stop`, speed-step and next-turn drafts and records local test copies | ROS delivery or controller acceptance |
| Combined companion | Selects directed starting lanes and red-line destinations, calculates a local A* route, interprets route chat and can display a read-only camera service | Localization, route execution or physical junction success |
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

These steps are sufficient for the chatbot, command preview and route planner. Docker,
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
6. For the combined map, chat and camera window, run:

   ```powershell
   py -3 laptop/duck2_companion.py
   ```

   Or double-click `laptop/Start-Duck2Companion.cmd`. Select a starting
   directed lane, then click a red marker as the destination. The local A*
   planner minimizes junction crossings and never sends its route draft.
   The Camera tab remains disconnected until **Start viewing** is pressed.

Try `Take the next right`, then `Actually, left`. **Record preview** stores a
local test record only. `Slow down a little` asks whether the controller's
standard fixed speed step should be used; it never silently converts “a little”
into a measured speed. Timed/distance stops, pauses, reverse, undo and
interrupt-with-cancellation are understood where possible but remain unavailable
for execution.

The combined companion is documented in
[docs/OFFLINE_COMPANION.md](docs/OFFLINE_COMPANION.md). Its route execution
button is intentionally disabled until junction traversal is physically
validated. Camera viewing uses a separate read-only ROS subscriber and owns no
publisher. The view worked during a stationary live check; starting a new view
still requires its separate robot service and local SSH tunnel for that session.

For a task-by-task guide covering all Windows apps, the read-only camera view,
connection checking and bounded test commands, see [docs/USAGE.md](docs/USAGE.md).

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
docker run --rm --network none --mount "type=bind,source=$repoPath,target=/project,readonly" --entrypoint bash altata-duck2:noetic-amd64 -lc 'source /environment.sh >/dev/null 2>&1; cd /project/tests && python3 -m unittest test_runtime_setup test_lane_follower test_navigation test_obstacles_and_connection test_first_test_readiness test_bounded_ground_supervisor test_steering_transition test_duck_avoidance test_ground_test_session test_chat test_offline_interpreter test_offline_chat_ui test_command_preview test_obstacle_language'
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

For a supervised physical lane test after the user gives a fresh `Go`, use the
bounded Windows wrapper:

```powershell
py -3 tools\run_duck2_ground_test.py `
  --label left-curve-to-straight --duration 15 --confirm-go
```

This command performs physical movement. Camera-guided tests accept up to 15
seconds (default 15), with an independent watchdog and unchanged wheel caps.
The runner verifies exclusive wheel ownership, saves a uniquely named recording
under `%LOCALAPPDATA%\Duck2\evidence`, and checks that normal control returns
afterward. It never deletes the robot-side recording. Do not run it for an
unobserved robot or without a new `Go`. Ordinary launchers remain unchanged.
The latest sharp-right bend completed successfully by user observation and
stopped promptly at the next red line. See the dated evidence in
[docs/TESTING.md](docs/TESTING.md).

Before an intersection run, record stationary red-line geometry at a measured
placement. This never releases the emergency stop and does not require `-Go`:

```powershell
.\tools\Start-Duck2-GroundTest.cmd `
  -Label red-line-10cm -InspectRedLine `
  -RedStopTriggerBottomFraction 0.65
```

After choosing the trigger from stationary 15 cm, 10 cm and 5 cm samples, one
supervised intersection test uses `straight`, `left` or `right`:

```powershell
.\tools\Start-Duck2-GroundTest.cmd `
  -Label junction-straight -Duration 15 `
  -JunctionTurn straight -RedStopTriggerBottomFraction 0.80 -Go
```

That command moves the robot and still requires a fresh user `Go`. The example
`0.80` value is illustrative until stationary samples establish the actual
threshold. Missing borders are expected only after the authorized red-line
departure; ordinary lane loss still stops the robot.

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

The current focused safety/navigation suite passed **150 tests**. The broader
offline suite passed **282 tests**, with four environment-specific skips. The
isolated ROS transport suite also passed in a disposable ARM64 container with
networking and hardware devices disabled. It covers camera freshness, shutdown,
heartbeat, gateway, recorder, route handling and unmarked authorized crossings.
Native Windows GUI checks remain separate commands listed above.

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
yellow divider during a dash gap. The former fixed-width white-only fallback
shifted the inferred centre to the opposite side and weakened the needed left
correction. Saved-frame replay confirms that the test-only measured-width
fallback removes those sign changes without changing colour thresholds or
both-boundary estimates. Straight driving, the left curve and the latest
sharp-right bend have supervised successful observations. Red stopping remains
farther from the line than desired; intersection traversal, obstacle avoidance,
routes and reverse remain unverified.
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
- Existing colour defaults detected both boundaries on straights but briefly
  entered white-only fallback during yellow-dash gaps. The bounded test preset
  now reuses the latest measured lane width across those gaps; ordinary
  launchers retain the previous default until the fix passes a supervised run.
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
