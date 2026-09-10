# Startup and reboot checklist

Use Windows PowerShell in the repository root. Examples use `py -3`; `.cmd`
shortcuts prefer this laptop's bundled Python, then `py -3`. Install Python
3.10+ with Tcl/Tk; enable the Windows Python launcher during installation. Use `py -3 -m pip install -r requirements-desktop.txt` for
Pillow camera support; install it in the interpreter actually used by the app.
The main companion and offline apps need no model API key.

## After switching on laptop and duck2

Use preparation and the companion from the same checkout. For the submitted
project, open the `AlTaTa - Duckietown Final` folder; a fresh clone works too.
Close an old companion before opening updated source. Closing requests Stop.

1. Enable the configured hotspot and allow duck2 2–5 minutes to boot.
2. Run `tools/Start-Duck2-Check.cmd`. SSH identity and login must pass. Exit 3
   means reachable but normal control is not ready; read its missing-node or
   ownership report. It does not automatically restart services.
3. Select one mode below. Opening an app/tunnel alone does not restore its
   backend after reboot. Start a physical route only while watching duck2.

The normal `laptop/Start-Duck2Companion.cmd` includes live chat by default;
special scenario launchers are optional. If the camera says **service reached,
but no fresh preview**, SSH/HTTP responded but the preview backend has no usable
frame. Keep the robot stopped and prepare the updated backend. A **connection
unavailable or timed out** message instead requires checking the robot,
connection check and SSH tunnel. Neither error calls for changing lane colours.

| Mode | Commands |
| --- | --- |
| Camera only | `tools/Start-Duck2-AppConnection.cmd`, then `laptop/Start-Duck2Companion.cmd` |
| Synthetic practice | `py -3 tools/interactive_bench.py`, wait BENCH READY, then `laptop/Start-BenchCompanion.cmd` |
| Physical track | `tools/Start-Duck2-DrivingMode.cmd`, then normal companion; confirm actual starting lane before Start |

Camera mode is driving-disabled with wheel output redirected. Simulation uses a
separate onboard ROS master with no hardware access; its window says SIMULATION.
Driving preparation stages current source and establishes exclusive ownership
while remaining stopped. It does not start a route. Fresh Go is required for
assistant-executed movement. Normal `car-interface` can remain stopped after
previous testing; do not restart it just to clear a warning on a floor/table.

## SSH

Key setup is one-time. If ssh-agent is disabled, use elevated PowerShell once:

```powershell
Set-Service ssh-agent -StartupType Automatic
Start-Service ssh-agent
```

Unlock the existing key in your normal Windows account when needed:

```powershell
ssh-add "$env:USERPROFILE\.ssh\duck2_ed25519"
ssh -o BatchMode=yes -o StrictHostKeyChecking=yes duck2 hostname
```

On a new laptop, verify the robot host key through trusted information first.
`tools/setup_duck2_ssh.ps1` then creates/loads a protected key and appends its
public half without removing existing access. It requires a previously trusted
key. Never disable verification. The robot account password, laptop Ubuntu
password, hotspot password and key passphrase are different credentials; none
belongs in the repository.

## Troubleshooting

| Symptom | Action |
| --- | --- |
| Cannot resolve duck2 | Check hotspot and boot completion |
| Permission denied/publickey | Unlock existing key; do not regenerate it |
| Host-key mismatch | Verify robot identity before retrying |
| SSH channel connection refused | Start the correct backend; tunnel alone is insufficient |
| Ports 8765/8766 occupied | Inspect and close only the old app SSH tunnel, then retry |
| Driving disabled | Stationary preview is active; physical track uses DrivingMode |
| Wheel requests but no motion | Stop; requested values do not prove executed motor output |
| Map/chat clipped | Reopen updated app, use Fit map or resize |
| Dashboard unhealthy, SSH/ROS healthy | Separate dashboard issue; do not reinstall ROS |

The connection launcher checks both command and camera services. The camera may
be reachable before receiving its first frame. In stationary preview run
`py -3 tools/check_stationary_camera.py` for decoding, timestamps and sampled age.

Docker Desktop is needed only for local image builds/tests. Its repeated stale
`dockerInference` socket failure on this laptop was repaired by stopping Desktop,
renaming only its runtime socket directories under local application data,
recreating those directories and restarting Desktop. Images/volumes were kept.
Diagnose logs before repeating repairs; do not factory-reset or prune routinely.

## Finish

Use STOP DUCK2 before closing a physical session. Stop only the temporary
preview/driving container you started and close its SSH tunnel. Synthetic
sessions have a deadline and cleanup. Do not restore normal driving control
automatically on a table/floor. [USAGE](USAGE.md) documents deliberate restoration.

For local checks: `py -3 -m pip install -r requirements-test.txt`, then
`py -3 tools/check_project.py --tests`. The ROS transport script requires an
isolated Noetic container; it is excluded from native unittest discovery.


## Choose the correct Python on a new or existing laptop

`py -3 -m pip install -r requirements-desktop.txt` targets standard Windows
Python. This laptop's `.cmd` shortcuts first look for the bundled interpreter
below. If it exists, install dependencies into it instead (PowerShell, repo root):

```powershell
$duck2Python = "$env:USERPROFILE\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
& $duck2Python -m pip install -r requirements-desktop.txt
& $duck2Python -c "import tkinter; from PIL import Image; print('Desktop dependencies ready')"
```

On another laptop where that file is absent, the shortcuts use `py -3`.
Do not install Windows GUI dependencies only into WSL Python. The project can
remain in WSL storage while Windows Python and Windows OpenSSH run its launchers.

For first-time SSH setup after independently trusting the robot host key:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tools\setup_duck2_ssh.ps1
```

This script needs the existing robot account password once to append the public
key. Enter it privately. The key passphrase unlocks your local key; the hotspot
password only joins Wi-Fi. Never paste any of them into source or documentation.
If identity is not yet trusted, the script stops; resolve that before rerunning.

After a disconnect/reconnect, enable the hotspot, unlock the key only if needed,
run the quick check and prepare the desired backend. Reinstalling keys, rebuilding
images and repeating full metadata inspection are not routine reboot steps.
