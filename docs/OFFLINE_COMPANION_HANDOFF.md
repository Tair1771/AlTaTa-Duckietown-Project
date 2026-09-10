# Offline companion handoff

Historical milestone. The current companion supports live control and local chat;
use [STARTUP](STARTUP.md), [LIVE_CHAT](LIVE_CHAT.md) and [TESTING](TESTING.md).
The offline-only restrictions below describe the earlier 2026-09-08 state.

Date: 2026-09-08

## Completed locally

- Added one shared five-junction map model for the ROS controller and Windows
  planner, including 16 direction-specific red-line approaches.
- Added a directed A* planner that minimizes junction crossings, forbids
  U-turns, derives local turn instructions and ends at the selected approach.
- Added an offline companion window with clickable right-lane starts, red-line
  destinations, highlighted routes, route-aware chat and a disabled execution
  control.
- Added a loopback-only camera client and a standalone read-only ROS camera
  service for Normal, Mask and Overlay views. The service reuses the exact lane
  detector method and owns no publisher. It now initializes as a Duckietown
  `DTROS` perception node before subscribing, as required by duck2's runtime.
- Extended set-route validation with a map version and exact endpoint metadata.

## Verification

- A* matched an independent breadth-first shortest-path result for all 256
  start/destination combinations.
- Offline route/chat/camera-client tests passed with HTTP mocked.
- The real Windows Tk window opened, planned, chatted, cleared and closed while
  socket and HTTP access were blocked.
- The complete pinned Noetic test run passed 228 tests with 3 environment skips
  using `--network none`, including detector parity and navigation regressions.
- Stationary duck2 verification passed through the Windows key-based SSH path.
  The temporary read-only gateway received fresh compressed-camera frames and
  the Windows companion displayed Normal, Mask and Overlay views with advancing
  timestamps and fresh ages. Its tunnel-disconnected state and automatic
  recovery after a replacement tunnel were also observed.
- The companion planned an offline `A->B` to `D->A` draft, applied a next-left
  correction, and retained a disabled route-execution control. The temporary
  gateway, copied files and tunnel were removed afterward; the camera node,
  kinematics node and wheel-driver publisher were restored to their original
  state.

## Deferred live checks

- Validate red-line identity assumptions and all junction manoeuvres on the
  course before enabling route delivery or the Start route control.
- The table scene produced the expected "Lane lost: no boundaries" diagnostic;
  it is not colour or lane calibration evidence. Verify camera masking and lane
  diagnostics again on the course before changing any perception settings.
- Automatic localization, obstacle passing, reverse and live chatbot delivery
  remain disabled.

Recommended next model: **GPT-5.6 Terra, Medium** for the stationary camera and
connection verification. Keep duck2 stationary with its camera uncovered.
