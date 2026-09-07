# Duck2: lane following and offline command previews

University Duckietown project for **duck2**. This repository contains our ROS 1
lane follower, an offline English request interpreter, a separate command-preview
app, and an existing connected desktop companion.

**Start with the offline preview app. It needs no API key, Docker, ROS, or bot
connection.** The robot code has passed synthetic and isolated ROS checks.
Camera calibration, motor response, stopping distance, and junction traversal
still require physical testing.

## What we currently have

| Component | Current behaviour | Limit |
| --- | --- | --- |
| Offline interpreter | Curated English requests, numbers, units, corrections, follow-up answers, greetings, and clarification | Describes requests only |
| Offline command preview | Basic stop, speed changes, and next-turn drafts; in-memory preview records | No delivery, movement simulation, or execution |
| ROS lane follower | OpenCV yellow/white lane detection and stamped left/right wheel commands | Needs our own camera and motor calibration |
| Stopping safeguards | Settings validation, camera freshness checks, lane-loss stops, steering reset, zero output on shutdown | Does not establish physical stopping distance |
| Experimental duck passing | Compact yellow candidates and optional left-pass/right-return feedback | Disabled by default; [calibration and limitations](docs/DUCK_AVOIDANCE.md) |
| Red-line / obstacle handling | Latched red stops and a provisional bright/coloured obstacle heuristic | Dark objects may be missed; not general recognition |
| Route / connected companion | Existing route state machine, HTTP gateway, controls, acknowledgments, and heartbeat handling | Placement and junction calibration required; physical operation unverified |
| Camera diagnostics / recorder | Lane overlays, stop reasons, actual published wheel values, and image capture | Real track recordings still needed |

The offline chatbot is a **rule-based parser with conversation memory**, not an
LLM. It understands a bounded collection of English phrases. It does not know
live robot status. Closing either offline app discards its conversation state.

Obstacle phrases such as "something in the way", "duck", and "go around it" are
understood by both offline windows. They remain interpretations with no recordable
avoidance command or robot delivery. See [duck avoidance](docs/DUCK_AVOIDANCE.md)
for the separate experimental controller, its parameters and required checks.

## How the parts connect

```text
Offline interpreter window -> interpretation -> reply only

Offline preview window -> same interpreter -> translation -> draft
                                                       -> Record preview
                                                       -> in-memory fake receiver

Existing connected companion -> HTTP gateway -> ROS commands -> controller

Compressed camera -> OpenCV lane follower -> stamped wheel commands
```

There is currently **no robot-delivery path from either offline app**.
Interpretation, a command preview, controller acceptance, and physical completion
are separate stages.

| Interface | Topic | Message |
| --- | --- | --- |
| Camera input | /${VEHICLE_NAME}/camera_node/image/compressed | sensor_msgs/CompressedImage |
| Wheel output | /${VEHICLE_NAME}/wheels_driver_node/wheels_cmd | duckietown_msgs/WheelsCmdStamped |
| High-level requests | /${VEHICLE_NAME}/lane_follower/command | std_msgs/String with JSON |
| Status | /${VEHICLE_NAME}/lane_follower/status | std_msgs/String with JSON |

Our robot name is `duck2`. The Duckietown runtime supplies `VEHICLE_NAME`.

## Setup A: offline chatbots on Windows

### 1. Download the project

Accept the GitHub invitation if the repository is private. Use **Code ->
Download ZIP** and extract it, or clone with Git. Replace the placeholder below
with the team's actual GitHub path:

```powershell
git clone https://github.com/YOUR-ACCOUNT/YOUR-REPOSITORY.git duckiebot-ros
cd duckiebot-ros
```

Use the team's repository, not the original Duckietown template. The repository
root contains this README, `Dockerfile`, and `laptop`. The commands in this
section run in **Windows PowerShell from that root**.

### 2. Install and check Python

Install Windows Python from [python.org](https://www.python.org/downloads/windows/).
Desktop tests were run with Python 3.12. Include Tcl/Tk support and the Python
launcher. See [Windows installation details](https://docs.python.org/3.12/using/windows.html).

Open a new PowerShell window:

```powershell
py -3 --version
py -3 -m tkinter
```

The second command should open a small Tk window; close it before continuing.
If `py` is unavailable but `python` works, substitute `python` for `py -3`.
If Tk is missing, repair/install Python with Tcl/Tk support; do not try to
install a package named tkinter with pip.

**Neither offline app needs pip packages or an API key.** The robot dependency
files are container inputs, not instructions to install ROS into Windows.

### 3. Open the app

For command drafts and local test records:

```powershell
py -3 laptop/command_preview_chat.py
```

Alternatively, double-click `laptop/Start-CommandPreview.cmd`. The title is
**Offline command preview — no robot connection**.

For interpretation only:

```powershell
py -3 laptop/offline_chat.py
```

Alternatively, double-click `laptop/Start-OfflineChat.cmd`. These launchers use
a locally bundled Python when available, otherwise `py -3`. Codex is not
required. If a launcher closes immediately, use PowerShell to see its error.

### 4. Try a conversation

1. Enter `Take the next right`.
2. Enter `Actually, left`. The current turn draft changes.
3. Click **Record preview**. A local left-turn test record appears.
4. Enter `Slow down a little`. The app asks whether to substitute the
   controller's standard speed step.
5. Answer `Use the standard step` to prepare that draft, or `Cancel` to clear it.
6. Use **Clear conversation** to reset context, drafts, and local records.

| Request | Preview result |
| --- | --- |
| Stop | Drafts stop |
| Speed up / Slow down | Drafts speed_up / slow_down |
| Take the next left/right/straight | Drafts turn with that direction; reports route/exit prerequisites |
| Speed up by 10 percent / A little faster | Clarification before substituting a standard speed step |
| Stop after two seconds / Stop after 30 cm | Can be interpreted; no executable preview mapping |
| Stop at the next red line / Stop for five seconds | Can be interpreted; no executable preview mapping |
| Reverse, undo, interrupt-and-cancel | No executable preview mapping |
| Negated, ambiguous, or unsupported request | Cannot leave a stale draft recordable |
| Greetings and thanks | Conversation only; may preserve an existing draft |

Speed steps multiply the current controller speed scale by 1.2 or 0.8, subject
to limits. They are not measured velocities. Turn previews do not prove that a
physical exit is available or calibrated. Recording consumes the draft and
preserves earlier records. Cancel clears the unrecorded draft/discussion; it
does not cancel robot movement.

### 5. Run chatbot tests

From the repository root in PowerShell:

```powershell
py -3 -X utf8 tests/test_offline_interpreter.py
py -3 -X utf8 tests/test_offline_chat_ui.py
py -3 -X utf8 tests/test_command_preview.py
py -3 -X utf8 tests/test_chat.py
```

Each should finish with `OK`. GUI tests require Tk and a desktop session.
`-X utf8` avoids Windows encoding errors when tests read source files.
The companion tests mock API responses; no API key is needed. The offline
window checks block network access and imports of connected code.

## Setup B: robot-code development without a bot

Skip this section if you only need the offline apps. Robot development uses
Ubuntu and Docker. The container supplies ROS, OpenCV, NumPy, and Duckietown
packages. Installation/builds need internet; the test containers below have
networking disabled.

### 1. Set up Ubuntu 22.04

On Windows, open **PowerShell as Administrator** and install WSL if needed:

```powershell
wsl --install -d Ubuntu-22.04
```

Restart if prompted, open Ubuntu, and create your Linux account. Check WSL mode:

```powershell
wsl --list --verbose
```

If Ubuntu-22.04 shows version 1, convert it:

```powershell
wsl --set-version Ubuntu-22.04 2
```

See [Microsoft's WSL commands](https://learn.microsoft.com/en-us/windows/wsl/basic-commands).
Native Ubuntu 22.04 can skip WSL and use its own Docker installation.

### 2. Enable Docker

On Windows, install/start Docker Desktop, use Linux containers and the WSL 2
engine, and enable **Settings -> Resources -> WSL Integration -> Ubuntu-22.04**.
See [Docker's WSL setup](https://docs.docker.com/desktop/features/wsl/).
Use this integration rather than installing a second Docker Engine in the same
WSL distribution.

In the **Ubuntu terminal**, check that client and server work:

```bash
docker version
docker run --rm hello-world
```

### 3. Create an Ubuntu checkout

The following commands are **Ubuntu Bash**, not PowerShell. On a new teammate's
machine:

```bash
sudo apt update
sudo apt install -y git python3-pip
mkdir -p ~/duckietown
cd ~/duckietown
git clone https://github.com/YOUR-ACCOUNT/YOUR-REPOSITORY.git duckiebot-ros
cd duckiebot-ros
chmod +x launchers/*.sh packages/duckie_lane_follower/src/*.py
```

Replace the GitHub path first. Private repositories require GitHub authentication
through a supported Git credential method. Alternatively, extract the ZIP into
this location; ZIP downloads omit Git history. The chmod command restores
executable permissions after browser uploads/ZIP transfers.

**If the working checkout already exists, use it instead of cloning over it:**

```bash
cd ~/duckietown/duckiebot-ros
```

Keep one working checkout per person and exchange changes through GitHub.
Windows File Explorer can open the WSL checkout at
`\\wsl.localhost\Ubuntu-22.04\home\YOUR-LINUX-USER\duckietown\duckiebot-ros`.

### 4. Build a laptop test image

For an Intel/AMD laptop, use an explicit local image tag:

```bash
docker build --build-arg ARCH=amd64 -t duck2-project:offline .
```

This uses the existing Dockerfile and Daffy ROS base. It is an **amd64 laptop
test image**, not an ARM robot deployment image. The first build downloads
dependencies and can take time. Require a successful build before continuing.
Rebuild after changes to robot source, launchers, or dependencies.

Keep the Dockerfile's `REPO_NAME=duckiebot-ros`: transport tests currently
expect the internal path `/code/catkin_ws/src/duckiebot-ros`.

### 5. Run robot checks in isolation

From the repository root in Ubuntu, after the build:

```bash
for suite in test_lane_follower test_navigation test_obstacles_and_connection test_first_test_readiness; do
  docker run --rm --network none \
    -v "$PWD:/project:ro" \
    --entrypoint python3 duck2-project:offline "/project/tests/${suite}.py" || break
done
```

All four suites should run and report `OK`. They use synthetic images, real
OpenCV, and mocked ROS interfaces. The loop stops if a suite fails.

Then run the actual ROS transport checks without robot connectivity:

```bash
docker run --rm --network none \
  -v "$PWD:/project:ro" \
  --entrypoint bash duck2-project:offline \
  -lc 'source /environment.sh >/dev/null 2>&1; python3 /project/tests/test_ros_transport.py'
```

This starts an isolated ROS master and synthetic camera inside the container.
It checks received wheel messages, timestamp rejection, camera loss, shutdown,
and existing route/gateway/recorder behaviour. It uses the **built** robot source,
so a stale image does not test your latest edits.

Older examples in `docs/TESTING.md` use the original local image tag
`duckietown/template-ros:v3-amd64`. Docker images are not included in a Git clone.
The explicit tag above avoids depending on a branch-derived image name.

## Setup C: physical testing when duck2 is available

WSL/Docker discovery, ROS connectivity, and GUI forwarding still need checking
on the actual network. Offline tests do not establish physical readiness.

### 1. Install Duckietown Shell if needed

Inside Ubuntu 22.04:

```bash
python3 -m pip install --user duckietown-shell
export PATH="$HOME/.local/bin:$PATH"
command -v dts
dts --help
```

Add the PATH export to `~/.bashrc` if needed in later terminals. Use the course's
established profile/distribution. This project has a **Daffy ROS base** and
template format 3. Follow the installed shell's profile prompts/help rather
than assuming an older `--set-version` flag still exists.
See [Duckietown Shell installation](https://github.com/duckietown/duckietown-shell).

Check installed build/run options:

```bash
dts devel build --help
dts devel run --help
```

The existing workflow uses `dts devel build -f`. A previous shell run reported
an unrecognized `v3` distro after packaging an image; see `docs/TESTING.md`.
Do not treat a nonzero exit as success. Resolve the course profile/image
configuration and build for the actual execution host before physical testing.
An amd64 laptop image does not prove an ARM build works.

In the installed tools, `-R duck2` selects the robot to connect to, while `-H`
selects the Docker host. Selecting a robot does not itself mean the container
is running on that robot. Verify the course's deployment arrangement.

### 2. Check connectivity and camera

Connect the laptop and powered bot to the course-configured network. In Ubuntu:

```bash
dts fleet discover
```

Stop discovery with Ctrl+C after checking for duck2, then:

```bash
ping -c 4 duck2.local
```

Confirm expected camera/wheel topics and message types, clock agreement, and
absence of another active driving controller. Hostname resolution alone is not
a ROS connectivity test. After building the appropriate image:

```bash
dts devel run -R duck2 -L lane-camera-debug -X
```

Inspect masks, lane centre, stop reasons, and actual wheel output while
stationary. **Camera-debug publishes only zero wheel commands.** The `-X`
GUI runtime must work on the laptop. Do not switch to a driving launcher to
work around missing camera windows.

### 3. Lifted-wheel check, then slow track check

Only after camera inspection, secure the bot on a stand with wheels clear:

```bash
dts devel run -R duck2 -L lane-follow-stand
```

This enables wheels: base `0.05`, wheel cap `0.07`, steering cap `0.02`.
Verify wheel/steering direction, lane-loss stopping, and Ctrl+C stopping.
Once those pass, test a short, supervised track segment:

```bash
dts devel run -R duck2 -L lane-follow
```

Road defaults are base `0.08` and cap `0.18`. These are normalized wheel
commands, **not metres per second**. Ordinary lane-follow mode latches red-line
stops. Keep Ctrl+C available and measure the bot's stopping distance.
Follow the [full physical-test checklist](docs/FIRST_TEST_READINESS.md).

### Other launchers and the connected companion

| Launcher | Purpose |
| --- | --- |
| lane-record | Camera/status recorder; requires a persistent capture-directory mount |
| lane-route | Prototype route mode; requires heartbeat, confirmed placement, and junction calibration |
| lane-chat | Route node plus HTTP gateway; requires a control token and the route prerequisites |
| default | Original template placeholder; does not start the lane follower |

Both route launchers keep `junctions_calibrated:=false`. Do not enable it before
measuring junction behaviour.

`laptop/Start-Duck2Chat.cmd` opens the **connected companion**, not an offline
app. Its direct controls do not need an LLM key, but model-backed conversation
does. It is outside the current API-free preview workflow. Connection requires
a running ROS controller/gateway, a reachable URL, and a matching
`DUCK2_CONTROL_TOKEN`. Do not commit that token or expose the control port to
the public internet. See [the command interface](docs/COMMAND_INTERFACE.md) and
[companion guide](laptop/README.md) before using it.

## Work remaining

1. **Collect our own camera evidence:** straights, curves, dashed lines, red
   approaches, shadows, glare, obstacles, and junction exits.
2. **Calibrate the physical bot:** colour/ROI settings, steering flip, motor
   response, stopping distance, and speed caps. Another identical bot's values
   do not establish our measured performance.
3. **Validate routes:** confirm map labels and starting pose, measure junction
   entry/turn/reacquisition, and test failure recovery.
4. **Improve language coverage:** use actual user examples to add phrases and
   regression cases; preserve clarification instead of guessing movement.
5. **Design future live chatbot integration separately:** distinguish delivery,
   acceptance, and completion; preserve stop/freshness gates; test in isolation
   before connecting. Offline drafts currently have no live delivery path.
6. **Defer unsupported controls:** measured-distance/time actions, pauses,
   reverse, interrupt-with-cancellation, and undo need separate implementation
   and physical validation. Understanding them is not executing them.
7. **Finish the handoff:** verify a fresh teammate installation, share the map
   and selected recordings explicitly, and record software versions and measured
   calibration values for submission.

## Repository guide

```text
packages/duckie_lane_follower/src/
  lane_follower_node.py       Perception, control, navigation, safeguards
  command_gateway.py         Existing HTTP-to-ROS bridge
  camera_capture.py          Camera/status recorder
launchers/                   Duckietown runtime presets
laptop/
  offline_interpreter.py     Rule-based interpreter and discussion memory
  offline_chat.py            Interpretation-only Tk window
  command_preview.py        Translation and in-memory fake receiver
  command_preview_chat.py   Offline preview Tk window
  duck2_chat.py, chat_core.py Existing connected companion
tests/                       Language, GUI, controller, ROS transport tests
docs/                        Requirements, interfaces, calibration/test notes
Dockerfile                   Duckietown ROS build
dependencies-*.txt           Container dependencies
.dtproject                   Duckietown project metadata
```

Detailed guides:

- [Offline preview behaviour](laptop/COMMAND_PREVIEW.md)
- [Interpreter examples and limits](laptop/OFFLINE_CHAT.md)
- [First-test preparation and unchanged HSV defaults](docs/FIRST_TEST_READINESS.md)
- [Robot verification results and environment limitations](docs/TESTING.md)
- [Commands, routes, gateway, and heartbeat](docs/COMMAND_INTERFACE.md)
- [Camera recording](docs/CAMERA_RECORDING.md)
- [Project/map assumptions](docs/PROJECT_REQUIREMENTS.md)
- [External interface references](docs/INTERFACE_REFERENCES.md)
- [Earlier local-model experiment](docs/LOCAL_MODEL_EXPERIMENT.md)

## Verification status, sharing, and attribution

Recorded on **2026-09-07**: 35 chatbot test methods passed on Windows, including
offline GUI lifecycle checks with network access blocked. Robot regression and
isolated ROS transport results are documented separately in `docs/TESTING.md`.
These results apply to the tested environment and inputs, not an untested
physical track. These setup instructions still need a clean-machine handoff check.

Upload source, launchers, dependencies, tests, documentation, and hidden project
metadata. Exclude credentials, .env, caches, runtime downloads, and generated
build outputs. Docker images, Python/WSL installations, chat history, and earlier
chat attachments do not transfer with a clone. Add the map/selected recordings
explicitly if needed.

This project started from [Duckietown's ROS template](https://github.com/duckietown/template-ros).
Preserve its licence material in `LICENSE.pdf`. The package manifest currently
declares MIT; reconcile the project-level licence/attribution before final
public release rather than assuming this README grants a new licence.
Other teams were consulted only for hardware/ROS interface facts, documented in
`docs/INTERFACE_REFERENCES.md`. Their movement/vision implementations and colour
values were not used for these changes.
