# Duck2 — lane navigation and live chat

The Windows companion combines a directed map, A* route planning, live camera
views and a local chatbot. The robot follows its right lane, stops at red lines
and receives one validated junction instruction at a time. Live chat supports
timed pauses, straight-road speed profiles and changes to future turns. It
requires no API key or model download.

**Start here:** [startup and reboot setup](docs/STARTUP.md),
[app operation](docs/USAGE.md), [chat commands and scenario](docs/LIVE_CHAT.md).

## Open the app and connect

Open this repository's folder in Windows Explorer or Windows PowerShell.
The same normal companion works from a Desktop checkout or a Windows-accessible
WSL checkout. Run preparation and the app from the same updated checkout;
already running apps and robot containers do not reload edited source files.

1. Enable the configured laptop hotspot, power on duck2 and allow it to boot.
2. Run `tools/Start-Duck2-Check.cmd`. Unlock the existing Windows SSH key if
   requested; key installation is a one-time step.
3. Choose the backend and app below. Keep duck2 stationary during preparation.
4. In the normal companion, click **Connect to duck2** and **Start viewing**.
   For movement, select the actual directed starting lane and destination red
   line, confirm placement and click **Start selected route** while watching.

| Use | Prepare | Open |
| --- | --- | --- |
| Map planning, offline | Nothing | `laptop/Start-Duck2Companion.cmd` |
| Stationary camera | `tools/Start-Duck2-AppConnection.cmd` | Normal companion |
| Track route with live chat | `tools/Start-Duck2-DrivingMode.cmd` | Normal companion |
| Optional multi-junction chat scenario | DrivingMode above | `laptop/Start-Duck2-JunctionChatCheck.cmd` |
| Pause check ending at the next red line | DrivingMode above | `laptop/Start-Duck2-PauseCheck.cmd` |
| Synthetic rehearsal | `py -3 tools/interactive_bench.py` | `laptop/Start-BenchCompanion.cmd` |

The scenario shortcuts connect and open the camera; **they do not press Start
or type messages**. A backend must already be prepared. Preview cannot drive.
Real command/camera URLs are `http://127.0.0.1:8765` and
`http://127.0.0.1:8766` through Windows SSH. Simulation uses separate
18765/18766 ports and a SIMULATION banner.

## Live-chat quick reference

| Type in Live chat | Meaning |
| --- | --- |
| `stop for 7s` | Request an immediate seven-second pause, then resume if healthy |
| `go straight at the next junction` | Replace future turns with straight; road curves still use lane following |
| `go left` / `go right` | Select the next legal junction exit |
| `left after that` | Append left to the future queue |
| `slow speed` / `normal speed` / `fast speed` | Select a confirmed-straight profile: 0.09 / 0.10 / 0.11 |
| `status` | Show the reported lane and pending turns |
| `stop` / `pause` | Pause; Continue is required within 30 seconds |
| `continue` | Resume a pause without skipping a red stop |
| `quit` or **STOP DUCK2** | End the run; no automatic resume |

These are normalized commands, not metric speeds. The deterministic parser
recognizes supported phrases and limited variants; unsupported wording asks
for clarification. It is not a trained language model. With Live chat enabled,
an empty queue at a red stop prompts for a direction and allows 60 seconds from
arrival. Map-only mode ends at its selected destination. See the
[complete command contract](docs/LIVE_CHAT.md).

## Installation and development

Windows Python 3.10+ with Tcl/Tk and Pillow is needed for the desktop app.
Use `py -3 -m pip install -r requirements-desktop.txt` with a standard Windows
Python installation. On this laptop the shortcuts prefer the bundled Python;
[STARTUP](docs/STARTUP.md) explains which interpreter receives dependencies.
Docker Desktop is needed for local ROS image builds and isolated ROS tests,
not for starting the source-mounted app service on duck2.

```powershell
py -3 tools/check_project.py
```

This checks Python syntax, documentation links and shortcut targets offline.
Install `requirements-test.txt` and add `--tests` for native regressions.
[ROBOT_SETUP](docs/ROBOT_SETUP.md) documents the pinned Noetic build and message
interfaces. [BENCH_CHAT_CHECKS](docs/BENCH_CHAT_CHECKS.md) covers isolated testing.

## Evidence and project files

Live chat is enabled by default in the normal companion; no scenario launcher
is required. The user reported the three-second pause check successful, and
individual road and intersection behaviours have accepted supervised runs.
The latest 2026-09-10 attempt did not curve enough and stopped. Full-route
physical validation remains **incomplete**. The software checks cover commands,
queues, camera previews and stopping; they cannot guarantee lane containment.
Keep these results separate in [TESTING](docs/TESTING.md).

Project authors: **Tair Kezdekbayev, Alvaro Mendez Li and Tan Kayra Erol**.
The [DuckietownTUM submission](https://github.com/DuckietownTUM/AlTaTa-Duckietown-Final)
and [personal mirror](https://github.com/Tair1771/AlTaTa-Duckietown-Final)
contain the same release. The [report PDF and LaTeX](report/README.md) include
all three authors.

| Folder/file | Purpose |
| --- | --- |
| `laptop/` | Companion, planner, local chat and retained legacy modules |
| `packages/duckie_lane_follower/` | ROS perception, navigation, gateways and watchdog |
| `launchers/` | ROS entry points; `default.sh` starts no application |
| `config/duck2.json`, `Dockerfile` | Verified runtime facts and pinned packaging |
| `tools/`, `tests/` | Connection, preparation, simulation and regression tooling |
| `docs/` | Current guides, interface descriptions and dated evidence |
| `report/` | Updated LaTeX report, PDF and build instructions |
| `LICENSE.pdf` | Existing applicable licence notice; retained |

[Documentation index](docs/README.md) ·
[Chatbot provenance](docs/CHATBOT_PROVENANCE.md) ·
[Careful cleanup record](docs/REPOSITORY_MAINTENANCE.md)

Credentials, camera recordings and recovery snapshots stay outside this
repository. Git operations are separate from setup and testing and occur only
when explicitly requested.
