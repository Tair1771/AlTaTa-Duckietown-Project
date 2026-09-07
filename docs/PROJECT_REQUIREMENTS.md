# Duck2 course project

## Sources and scope
Reviewed the team's Duckietown Presentation (7 pages), getting-started guide
(14 pages), Introduction lecture (40 pages), and extracted the supplied Python,
ROS, and computer-vision lecture text. Source PDFs are in the Windows Desktop
School/Duckietown folder. They remain unchanged.

Introduction slide 19 lists documented code, project report, peer feedback,
presentation and Demo Day. It weights the report 75% and presentation 25%.
The supplied materials do not provide a numerical driving rubric.
The team presentation explicitly includes obstacle response as well as lane
following, stable steering and high-level commands.

## Acceptance evidence still required
- Right lane: real recordings showing yellow on the left and white on the right,
  including dashed segments, both directions of curves, and partial line loss.
- Smooth movement: wheel telemetry and video at different camera frame rates.
- Red lines: approach and stop before crossing from every relevant direction;
  test false positives, lighting variation and a line remaining visible.
- Turns: complete left/right/straight junction traversals, then reacquire the
  correct outgoing lane. Curve following alone does not establish this.
- Predetermined route: establish initial junction and heading, execute the
  complete route, confirm each junction transition; do not count curves as junctions.
- Windows chatbot: paraphrases and contextual follow-ups affect live speed and
  the next eligible turn; show accepted/rejected commands and current bot state.
  A keyword-only parser would not fulfill contextual understanding.
- Obstacle response: test rubber ducks and other course obstacles with recorded
  camera images; define conservative stopping before attempting bypass maneuvers.
- Faults: debug mode, Ctrl+C, stale camera and connection loss stop output.
- Course deliverables: document measured results and limitations; report and
  presentation must not label synthetic tests as real-bot demonstrations.

## Map interpretation (from the supplied screenshot)
Five junctions are visible. Coordinates below identify schematic locations,
not calibrated physical poses:
A: left middle T junction.
B: central middle four-way junction.
C: right middle T junction (north, south and west exits).
D: right upper T junction.
E: bottom middle T junction.
Connections: A-B, B-C, B-D (curved approach), C-D, B-E,
A-E (lower-left perimeter), C-E (lower-right perimeter),
A-D (upper perimeter with the S bends).
Each connection contains separate lanes for opposing directions.
A candidate initial route is the outer perimeter A-D-C-E-A, starting at A
heading north. D, C, E and A are straight-through choices along this route.
This is a proposed route, not an observed robot location or executed plan.
The screenshot does not establish starting pose, dimensions, camera calibration
or junction markers. Route tracking must be reset explicitly after manual relocation.

## Implementation status
The ROS camera -> OpenCV -> WheelsCmdStamped architecture is preserved.
The node includes red-line stops, acceleration limits, elapsed-time steering
smoothing, route topology, route-aware turn overrides, bounded junction traversal
and visual outgoing-lane reacquisition. It also stops for provisional obstacle
candidates and expired desktop heartbeats. Default lane-follow modes retain the
persistent red stop. Route mode requires explicit starting-position confirmation;
junction execution also requires an explicitly enabled calibration setting.
With calibration enabled, route mode observes the stop dwell and continues
autonomously unless manually stopped. It stops at the route's final junction.

The Windows native chat client uses a contextual language model, conversation
history and live status, then sends validated high-level commands through an
HTTP-to-ROS gateway. Direct Stop/Continue/Slower/Faster controls require no model.
The API interpreter has not been evaluated live without a key. A small local
model was evaluated and rejected for inadequate contextual accuracy. The model never
receives direct wheel-control access. Late requests cannot supersede a newer
stop or silently apply to a different next junction.

Verification uses synthetic images with real OpenCV, plus actual ROS transport
and an isolated HTTP gateway. It establishes software behavior, not accurate
physical pose, stopping distance, correct turn angle or lap completion.
The bounded turn profile is provisional. Reacquisition can mistake an incorrect
lane for the intended outgoing lane. The map index assumes each red detection
corresponds to the expected junction; there is no marker-based localization.
These limitations require camera recordings and physical calibration.

## Next implementation steps
1. Complete and preserve the desktop-to-ROS integration checks.
2. Configure and evaluate the API-backed contextual language model on paraphrases,
   corrections, ambiguous references and requests made during junction changes.
3. Calibrate and improve provisional obstacle perception using course-relevant camera examples.
4. Gather real recordings for lane, stop-line and junction calibration.
5. Validate one physical lap, turn overrides, connection faults and recovery.
6. Produce report/demo evidence from measured results, not synthetic tests alone.
