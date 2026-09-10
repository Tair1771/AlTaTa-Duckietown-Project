# Combined route companion

`laptop/duck2_companion.py` combines the course map, camera panel and live
high-level route controls. It uses Python's standard library on Windows and
needs no API key. Route calculation still works with no robot connection.

## Route planning

The map has five provisional junctions, A through E, taken from the supplied
course image. A directed segment such as `A->B` means the bot is somewhere on
the right lane from A toward the red line before B. Exact distance along that
segment is deliberately not claimed.

Select **starting lane** and click an arrowed lane, then select **destination
red line** and click a red marker. The comboboxes provide the same selections
when markers overlap visually. Opposite approaches to one junction are
different destinations.

The map automatically fits the available panel when the window is resized,
including its line widths and labels. **Fit map** repeats that fit without
changing your route. Selecting the first starting lane automatically switches
clicks to destination selection; select **Start lane** again to change it.
The entire course stays visible without scrolling. The route preview scrolls
independently for longer routes. Placement confirmation, Start and STOP DUCK2
are available on the map tab as well as the camera/status tab. Changing either
endpoint clears the placement confirmation so an earlier route's confirmation
cannot accidentally authorize the new selection.

Close and reopen `laptop/Start-Duck2Companion.cmd` after an app update; an
already open window continues running its old code. Close while the robot is
stationary, since closing the app requests Stop. At display scaling above
150%, the minimum window size increases to keep the route controls visible.

The local A* state is `(previous junction, junction being approached)`. Legal
successors cross that junction without reversing, and each crossing costs one.
The heuristic is the relaxed junction distance, so the result is optimal for
fewest junctions. It is not a shortest-distance or fastest-time claim. Local
entry and exit ports produce the LEFT, RIGHT and STRAIGHT instruction list;
ordinary road curves do not consume instructions.

Map choices remain local until the user connects to `lane-continuous`, confirms
duck2's physical directed lane and presses **Start selected route**. The app
then sends the route and a separate Continue through the high-level gateway.
It never sends wheel values and refuses Start if wheel ownership is not
exclusive. Map-only mode ends at the destination red line. Live-chat mode waits
there for a further instruction, with a 30-second run-ending timeout.

## Live chat

The combined app now provides local live chat in the control panel's **Live
chat** tab. It validates future turns on the map and supports straight-only
speed and pause controls. See [LIVE_CHAT.md](LIVE_CHAT.md) for setup, examples,
queue ownership and timeout semantics. The separate offline interpreter remains
available for preview-only conversation, including:

- `I am starting on A to B`
- `Go to red line C from B`
- `Actually, take the next left`
- `Where are we going?`
- `Cancel route`

These phrases do not control the live route. Use the map selectors to plan and
the dedicated **STOP DUCK2** button to stop the live controller.

## Camera panel

The app starts disconnected and accepts only a loopback HTTP address. The
verified session setup is an SSH tunnel to the separate `lane-camera-view`
service. That service subscribes to compressed camera frames, calls the lane
follower's existing detector and exposes Normal, Mask and Overlay views at up
to five UI refreshes per second. It drops older queued frames and marks an age
above 0.5 seconds as stale.

The service owns no ROS publisher and contains no wheel topic or command
endpoint. Do not start a driving launcher merely to view the camera. The exact
commands are in [USAGE.md](USAGE.md#read-only-camera-view). A stationary live
view was verified. Route execution requires the separately prepared continuous
driving session and an explicit Start; opening the viewer alone does not drive.

## Offline checks

From the repository root on Windows:

```powershell
py -3 -m unittest tests.test_route_planner tests.test_companion_core `
  tests.test_camera_client tests.test_companion_ui
```

The full ROS test suite runs inside the pinned local image with Docker network
disabled. Details remain in `docs/TESTING.md`.
