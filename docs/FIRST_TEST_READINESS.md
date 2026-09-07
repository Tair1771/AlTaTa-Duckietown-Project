# First physical test preparation

The software checks in this update use our existing node and synthetic images.
They do not establish physical lane keeping, reliable shadow rejection, stopping
distance, correct turn timing or general obstacle detection. The offline chatbot
remains interpretation-only. No commands, reverse controls or undo were added.

## What changed

- Invalid settings fail before the camera subscriber starts. Driving flags must
  be booleans, numeric settings must be finite numbers, image regions must be
  ordered and within bounds, and lane-loss speed must be zero. Base speed must
  not exceed the wheel cap, and the wheel cap cannot exceed 1.
- Images need a positive ROS header timestamp that increases between accepted
  images. Images over 0.5 seconds old or over 0.1 seconds ahead of ROS time are
  rejected. Queueing and processing time count toward the freshness limit;
  source age also reduces the remaining watchdog budget. A rejected frame stops
  output immediately and cannot advance a route or establish obstacle clearance.
- Image processing runs independently of the wheel-command lock, so Stop,
  shutdown and the watchdog can still stop output during processing. Perception
  state is committed only after a successful second freshness check.
- Stopping or losing the lane clears steering history. Fresh-lane recovery uses
  the existing acceleration limit. Red and obstacle stops retain their latches;
  camera failure inside a junction still requires resetting the route position.
- The preview shows lane/red search regions, detected boundary centres, the
  estimated lane centre, single-boundary fallback or lane-loss explanation, stop
  reason, and actual published wheel values. Empty-lane images still show these
  diagnostics. Invalid image errors appear in logs and status.

Status retains all previous fields and adds `camera_valid`, `camera_error`,
`lane_diagnostic`, `lane_error`, and `stop_reason`. `camera_valid` is a freshness
check result, not evidence that the lane or physical robot is safe to drive.

## Colour calibration parameters

These exact bounds were already in our node. No new colour measurements have
been made and no other team's values were used. Each parameter is a list of three
integers in OpenCV HSV order; lower bounds cannot exceed upper bounds. The
accepted ranges are H=0..180 and S/V=0..255, retaining the previous upper limits.

| Private ROS parameter | Unchanged default |
| --- | --- |
| `yellow_lower` / `yellow_upper` | `[24, 140, 120]` / `[36, 255, 255]` |
| `white_lower` / `white_upper` | `[0, 0, 170]` / `[180, 55, 255]` |
| `red_low_lower` / `red_low_upper` | `[0, 110, 90]` / `[10, 255, 255]` |
| `red_high_lower` / `red_high_upper` | `[170, 110, 90]` / `[180, 255, 255]` |

The existing private ROI and steering parameters remain available. Set parameters
at launch time and restart to apply changes; this update does not add live tuning.
For a ROS command-line array override, the syntax is `_yellow_lower:='[24,140,120]'`.
Use the defaults initially and adjust only against your own recorded camera
images, retaining separate images to check the result. The existing weighted
mask detector and single-line fallback remain heuristics. Large coloured objects
or glare may still shift a detected boundary; low light may remove it entirely.

## When duck2 becomes available

1. Rebuild this repository for the bot's architecture. Confirm the installed ROS
   camera and wheel topic/message types and check for competing wheel publishers.
2. Start `lane-camera-debug` with the bot stationary. It publishes only zero wheel
   commands. Inspect actual images of straights, both curves, dashed lines, red
   lines, shadows and objects. Use the existing camera recorder to save examples.
3. Resolve camera timestamp errors before driving. Verify the camera's ROS clock
   and the node's ROS clock agree; do not bypass timestamp checks. Recorded old
   images need fresh publication timestamps for synthetic ROS transport tests.
   Restart the node after a backward ROS clock reset so old timestamp history
   cannot indefinitely reject the new clock timeline.
4. Lift the wheels and use `lane-follow-stand`: base 0.05, wheel cap 0.07 and
   steering cap 0.02. Confirm forward wheel direction, steering response, and
   Ctrl+C stopping. `flip_steering` still needs this physical confirmation.
5. Test a short straight at the existing conservative road settings (base 0.08,
   cap 0.18), then curves and red-line approaches. Record motor response and
   stopping distance before enabling calibrated junction traversal. These wheel
   settings are power fractions, not metres per second.

No tuning can guarantee a first successful lap without these physical checks.
Dark obstacles remain a known gap in the pre-existing obstacle heuristic.
