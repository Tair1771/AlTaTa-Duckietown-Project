# Duck2 usage guide

This guide separates the apps that are safe to use anywhere from commands that
require duck2 to be observed on the course. None of the Windows offline apps
need an API key.

## What to start

| Goal | Windows command or file | Connects to duck2? | Can move duck2? |
| --- | --- | --- | --- |
| Understand plain-English requests | `laptop/Start-OfflineChat.cmd` | No | No |
| Preview basic high-level commands | `laptop/Start-CommandPreview.cmd` | No | No |
| Plan a route, chat and view a camera stream | `laptop/Start-Duck2Companion.cmd` | Only after starting its viewer | No |
| Check robot connection and normal ROS ownership | `tools/Start-Duck2-Check.cmd` | Yes, read-only | No |
| Inspect a red line while stationary | `tools/Start-Duck2-GroundTest.cmd -InspectRedLine` | Yes | No |
| Run a supervised test | `tools/Start-Duck2-GroundTest.cmd ... -Go` | Yes | Yes |

Run the `.cmd` files by double-clicking them, or open Windows PowerShell in the
repository root and run the equivalent command with `./`.

## Offline chatbot and command preview

The two offline chat windows use only Python's standard library and Tk. Start
them from the repository root:

```powershell
py -3 laptop/offline_chat.py
py -3 laptop/command_preview_chat.py
```

The first explains what it understood. The second can make local drafts for
`stop`, `speed_up`, `slow_down`, and the next `left`, `right`, or `straight`
turn. **Record preview** saves only an in-memory fake-receiver record. Neither
window makes a network request or sends a ROS command.

## Combined companion

Start the companion with:

```powershell
py -3 laptop/duck2_companion.py
```

The **Map & route** tab asks for a directed starting lane and a red-line
destination. It calculates a right-lane-only A* route with the fewest junction
crossings. The route is a draft: the disabled **Start route** control cannot
deliver it to duck2.

The **Camera & chat** tab keeps chat and camera together. Chat is offline and
can describe a route or revise its next turn. The Normal, Mask and Overlay
camera choices are read-only views. They do not create a wheel publisher or
enable driving.

## Read-only camera view

The camera app expects `http://127.0.0.1:8766`. Its service must be running on
duck2 first. From a prepared Duckietown development session, launch only the
read-only service:

```powershell
dts devel run -R duck2 -L lane-camera-view
```

In a second Windows PowerShell, create the local-only tunnel and leave that
window open while viewing:

```powershell
ssh -N -L 8766:127.0.0.1:8766 duck2
```

Then start the companion, open **Camera & chat**, keep the default local
address, and select **Start viewing**. Choose **Normal**, **Mask**, or
**Overlay**. Select **Stop** before closing the tunnel. The service subscribes
to the compressed camera topic and exposes no command endpoint. Do not run a
driving launcher merely to use the camera.

If the app reports that the camera connection is unavailable, first run
`tools/Start-Duck2-Check.cmd`; confirm the robot has booted and the service is
still running. Never replace the loopback address with duck2's hotspot IP.

## Connection check

Turn on the laptop hotspot, then duck2, and allow two to five minutes for the
robot to boot and join the hotspot. Run:

```powershell
tools/Start-Duck2-Check.cmd
```

The check verifies the saved SSH identity and key login, ROS master, essential
containers, and normal wheel publisher. It does not view images, publish wheels
or call motor services. If the key agent was restarted, unlock the dedicated
key with `ssh-add $env:USERPROFILE\.ssh\duck2_ed25519` and enter only the key
passphrase.

## Track-test commands

Every movement test requires duck2 upright, on the track, observed by a person,
with the cable slack and clear of the wheels. The bot must be stationary before
you add `-Go`.

First, collect stationary red-line samples at measured 15 cm, 10 cm and 5 cm
front-to-line positions. This keeps the emergency stop latched and does not
move duck2:

```powershell
.\tools\Start-Duck2-GroundTest.cmd -Label red-line-10cm -InspectRedLine
```

After choosing the provisional red threshold from those samples, the next
planned moving test is one straight four-way crossing:

```powershell
.\tools\Start-Duck2-GroundTest.cmd -Label junction-straight -Duration 15 `
  -JunctionTurn straight -RedStopTriggerBottomFraction 0.80 -Go
```

`0.80` is only an example until the stationary samples establish the correct
threshold. A junction test permits missing lane borders only during its bounded
authorized crossing and reacquisition phases. It still stops on stale camera
data, controller faults, wheel stalls, ownership conflicts, manual Stop or its
deadline.

Details of all status fields, command semantics and safety gates are in
[COMMAND_INTERFACE.md](COMMAND_INTERFACE.md). Measured history and remaining
physical work are in [TESTING.md](TESTING.md).
