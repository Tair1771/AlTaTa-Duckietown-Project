# Using the companion

## Connect and start

1. Follow [STARTUP](STARTUP.md). With duck2 stationary, run
   `tools/Start-Duck2-DrivingMode.cmd`. It stages current source, suspends
   `car-interface`, establishes sole wheel ownership and awaits a route at zero.
2. Open `laptop/Start-Duck2Companion.cmd` from the same checkout.
3. In **Camera & status**, Connect to duck2 and Start viewing. Control uses
   `http://127.0.0.1:8765`; camera uses `http://127.0.0.1:8766` through SSH.
4. In **Map & route**, choose the actual directed starting lane and destination
   red-line approach. A* minimizes legal junction crossings using right-hand
   lanes. Fit map and resizing keep the complete map visible.
5. Leave **Live chat (ask when queue ends)** checked for chat control, or
   uncheck it for map-only operation ending at the selected destination.
6. Confirm placement outside the intersection and press **Start selected route**
   while watching. Keep the camera uncovered and any charging cable slack.

Position follows the selected start and confirmed turns, not global localization.
Picking up or repositioning duck2 requires Stop and a new start confirmation.
Displayed wheel values are requests, not proof of motion.

## Camera and chat

Normal shows the camera image, Mask shows lane colours, and Overlay adds geometry
diagnostics. Camera age is distinct from frame rate. Preview and controller are
separate services; a recovered preview does not clear an onboard fault.

Open **Live chat**, type a message and press Enter or Send. Shift+Enter adds a
line. Commands are not prefilled. `stop for 7s` pauses immediately on acceptance
and resumes if healthy. `go straight at the next junction` changes the next
red-stop decision; subsequent road curves retain lane following.

Red lines have a two-second dwell. When its future queue is empty, live chat
asks for a direction and waits up to 60 seconds from arrival. Map-only mode
ends at its destination. See [LIVE_CHAT](LIVE_CHAT.md) for all commands.

## Finish and recover

**STOP DUCK2** or `quit` ends the run and cancels automatic resume. Closing the
app requests Stop. Reconnect is blocked during Start and active live sessions.

To reload source or shut down, first Stop, confirm the robot is stationary and
close the app. Then remove only the project container:

```powershell
ssh duck2 docker stop -t 10 duck2-companion-driving
ssh duck2 docker rm duck2-companion-driving
```

Prepare again to load updated source. Normal `car-interface` remains stopped.
Restore it only after the project container has exited, output is zero, and
normal control is intended:

```powershell
ssh duck2 docker start car-interface
tools/Start-Duck2-Check.cmd
```

Preview-only cleanup uses `duck2-companion-preview`. Do not indiscriminately
restart robot services. Freshness checks, red stopping, bounded unmarked
maneuvers and the independent watchdog are required during normal operation.
