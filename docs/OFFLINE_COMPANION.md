# Offline combined companion

`laptop/duck2_companion.py` combines the course map, offline chatbot and a
read-only camera panel. It uses Python's standard library on Windows and needs
no API key. Route calculation works with no robot or network connection.

## Route planning

The map has five provisional junctions, A through E, taken from the supplied
course image. A directed segment such as `A->B` means the bot is somewhere on
the right lane from A toward the red line before B. Exact distance along that
segment is deliberately not claimed.

Select **starting lane** and click an arrowed lane, then select **destination
red line** and click a red marker. The comboboxes provide the same selections
when markers overlap visually. Opposite approaches to one junction are
different destinations.

The local A* state is `(previous junction, junction being approached)`. Legal
successors cross that junction without reversing, and each crossing costs one.
The heuristic is the relaxed junction distance, so the result is optimal for
fewest junctions. It is not a shortest-distance or fastest-time claim. Local
entry and exit ports produce the LEFT, RIGHT and STRAIGHT instruction list;
ordinary road curves do not consume instructions.

The route is an offline draft. The payload includes `offline_only: true`,
`execution_enabled: false` and `position_confirmed: false`. The app has no
control transport and the **Start route** button is disabled.

## Chat

The combined app reuses the curated offline interpreter and adds route phrases:

- `I am starting on A to B`
- `Go to red line C from B`
- `Actually, take the next left`
- `Where are we going?`
- `Cancel route`

A next-turn correction recalculates a legal route to the same exact red-line
destination. An unavailable exit asks for clarification and invalidates the
recordable draft. Stop, speed, reverse and obstacle language remain
interpretations only; no simpler command is silently substituted.

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
view was verified; route execution and camera-based driving are still disabled.

## Offline checks

From the repository root on Windows:

```powershell
py -3 -m unittest tests.test_route_planner tests.test_companion_core `
  tests.test_camera_client tests.test_companion_ui
```

The full ROS test suite runs inside the pinned local image with Docker network
disabled. Details remain in `docs/TESTING.md`.
