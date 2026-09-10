# Operate the companion

Use [STARTUP](STARTUP.md) for installation and reboot recovery. All commands
below run from the existing repository in **Windows PowerShell**. The app and
its current local live chat do not require an API key.

## Choose one mode

| Goal | Prepare | App |
| --- | --- | --- |
| Offline map planning | None | `laptop/Start-Duck2Companion.cmd` |
| Stationary real camera | `tools/Start-Duck2-AppConnection.cmd` | Normal companion |
| Physical route and live chat | `tools/Start-Duck2-DrivingMode.cmd` | Normal companion |
| Current combined chat scenario | DrivingMode | `laptop/Start-Duck2-JunctionChatCheck.cmd` |
| Pause-only scenario | DrivingMode | `laptop/Start-Duck2-PauseCheck.cmd` |
| Simulated app practice | `py -3 tools/interactive_bench.py` | `laptop/Start-BenchCompanion.cmd` after BENCH READY |

Preview is driving-disabled. Simulation uses an isolated ROS master with no
hardware access. Running a GUI alone does not start its backend. Do not run
preview and driving backends together on the same ports/ROS node names.

## Start a continuous session

1. Power on duck2 on the configured hotspot. Run `tools/Start-Duck2-Check.cmd`.
   A working SSH connection does not establish wheel ownership or scene quality.
2. Keep duck2 stopped and run `tools/Start-Duck2-DrivingMode.cmd`. It uploads
   current Python source, uses the installed pinned ARM64 base, stops project
   preview and normal `car-interface`, and prepares the temporary
   `duck2-companion-driving` container. It verifies exclusive wheel ownership,
   zero requested/executed output and `awaiting_route` before releasing a prior
   test emergency-stop latch. It sends no route or Continue. No local Docker
   Desktop build is needed.
3. Open `laptop/Start-Duck2Companion.cmd`. On **Camera & status**, connect to
   `http://127.0.0.1:8765` and start viewing `http://127.0.0.1:8766`. These are
   local SSH tunnel addresses, not the robot's hotspot address.
4. On **Map & route**, choose the actual directed starting lane and destination
   red line. A* chooses the fewest legal junction crossings in the directed map.
   Use Fit map/resize as needed. Physical placement determines the starting state;
   there is no automatic global localization.
5. Leave **Live chat (ask when queue ends)** checked for chat control, or uncheck
   it to finish a map-only route at the destination. Confirm placement and press
   **Start selected route** while observing duck2.
6. Type commands in **Live chat** and press Enter/Send message. Shift+Enter adds
   a line. The user sends messages; the app does not preload a test script.
   See [LIVE_CHAT](LIVE_CHAT.md) for exact supported phrases and deadlines.

Start is a high-level request. Displayed wheel values are requests, not proof
of motion. Status and camera views are separate services; a healthy preview
does not prove the controller has stayed healthy throughout a run.

## Optional chat scenarios

The normal companion above is the complete app, with live chat enabled by
default and no scenario-specific completion limit. The shortcuts below are
optional rehearsals. They do not replace the normal launcher.

The initial-straight check selects A → E → B and ends at the red line after
one junction. Sending `go straight at the next junction` changes its finish to
C. See [the scenario options](LIVE_CHAT.md) before choosing a test shortcut.

### Multi-junction rehearsal

Place duck2 after the curve on the A → E straight, centred in the right lane.
Open `laptop/Start-Duck2-JunctionChatCheck.cmd` after preparing DrivingMode.
It selects A → E → B → C, connects and shows the camera but does not start.

1. Confirm placement and click Start. While moving, type `stop for 7s`.
2. During/before the end of the pause, type `go straight at the next junction`.
   This replaces the future left/right queue with one straight crossing at E.
3. The controller stops/dwells at E, crosses toward E → C, then returns to road
   lane following, including the left curve. Straight is a junction instruction,
   not a command to hold straight steering over the next road segment.
4. At C it asks for a direction and waits up to 60 seconds from red-stop arrival.
   Type `go left`; the legal outgoing approach is C → B.
5. The scenario follows that lane and ends at the C → B red line.

The scenario is implemented. The latest physical attempt ended because the
battery depleted; it remains **incomplete, not validated as a full sequence**.
Do not resume an old run after picking up the robot: reset its directed start.

## Camera and status

Normal shows the image; Mask shows detected colours; Overlay adds detector
diagnostics. The normal companion needs explicit Connect and Start viewing;
scenario shortcuts connect automatically. Camera viewing does not send motion.
If a controller fault ends a junction, a fresh-looking preview afterward does
not invalidate that fault. Check the recorded stop reason before changing colour
thresholds. The camera timeout is retained; preview processing is rate-limited.

## Pauses, Stop and recovery

`stop for 7s` requests zero as soon as the robot accepts it and resumes after
seven seconds if healthy. Transport/braking have nonzero latency. The pause
preserves the queue. Bare `stop` pauses for up to 30 seconds awaiting Continue.
At an empty-queue red stop, the separate direction timeout is 60 seconds.
**STOP DUCK2** or `quit` ends the run and cancels automatic resume.

Closing the app requests Stop; heartbeat loss also stops the controller.
Camera freshness, watchdog, ownership, stall and maneuver deadlines remain
active. After a fault or Stop, confirm physical placement and start a new run.
An old Go does not authorize an assistant to start another physical run.

## Finish and recover

1. Press **STOP DUCK2** and confirm the robot is physically stopped.
2. Close the app. Stop only the temporary project container. The commands below
   are deliberate cleanup, not a task to run automatically on every connection:

```powershell
ssh duck2 docker stop -t 10 duck2-companion-driving
ssh duck2 docker rm duck2-companion-driving
```

Confirm temporary publishers have exited and wheel output is zero. Restore
normal `car-interface` only when normal controller operation is intended and
cannot resume unwanted motion. Then use `ssh duck2 docker start car-interface`
and `tools/Start-Duck2-Check.cmd` to inspect normal ownership. Otherwise leave
normal control stopped. Close only the project SSH tunnel when finished.

A new driving preparation starts stopped and refuses an unsafe replacement.
If preparation fails after suspending normal control it leaves it stopped.
Do not restart containers indiscriminately to clear a warning.

For preview-only cleanup, stop/remove `duck2-companion-preview` instead; see
[APP_CONNECTION](APP_CONNECTION.md). For source changes: Stop, close the app,
prepare again while stopped, reopen, confirm the actual starting lane.

## Other tools

[BENCH_CHAT_CHECKS](BENCH_CHAT_CHECKS.md) covers track-free rehearsal.
[HISTORICAL_BENCH_SETUP](HISTORICAL_BENCH_SETUP.md) preserves old bounded trial
commands; they are not the current app workflow. Earlier offline interpreters
remain runnable directly with Python; see [laptop/README](../laptop/README.md).
