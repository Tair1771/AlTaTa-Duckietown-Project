# Duck2 Windows applications

Current startup/reboot instructions: [STARTUP](../docs/STARTUP.md). For isolated
interactive practice, start `tools/interactive_bench.py` from the repository
root, then open `Start-BenchCompanion.cmd`. It uses separate simulation ports
and cannot access the physical wheel driver. [Bench guide](../docs/BENCH_CHAT_CHECKS.md).
Install `requirements-desktop.txt` in your selected interpreter for Pillow camera
resizing and checks; the basic UI works with Python/Tk alone.

For current work, double-click `Start-Duck2Companion.cmd`. It opens a combined
map, camera viewer and live high-level route controller without an API key.
Choose a directed lane for the starting direction and a red marker for the
destination. The laptop calculates the route with A*, minimizing junction
crossings. The local **Live chat** panel edits the future turn queue, requests
straight-only speed profiles and pauses, and reports completed turns. No API key
is needed. See [LIVE_CHAT.md](../docs/LIVE_CHAT.md). After the separately started continuous
ROS service is connected, confirm the physical starting lane and press **Start
selected route**. The app sends only high-level instructions, never wheel values.

The Camera tab accepts only a loopback URL, normally
`http://127.0.0.1:8766`, reached through the documented SSH tunnel. Opening
the application does not connect; press **Start viewing** explicitly. The
companion uses Python/Tk with optional Pillow. See [USAGE](../docs/USAGE.md) for the
continuous launcher, exclusive-control step, tunnel and app instructions.

## Earlier connected companion (not needed for the current project workflow)

Double-click Start-Duck2Chat.cmd on this laptop, or run duck2_chat.py with Python 3 and Tkinter on Windows. It uses only standard
Python libraries. The code can be run directly from the WSL repository using
its Windows network path; no second code copy is needed.

Use lane-chat for the combined node and gateway. Connect with its URL and control token. The gateway
must be running with the lane follower and receiving fresh status. Connecting
alone does not start the robot. Direct controls are available without an API key.

For contextual chat, enter an OpenAI API key in the masked field before Connect
(or set OPENAI_API_KEY in the launching process). It is retained only in memory.
The default model is gpt-5.4-mini; override with DUCK2_CHAT_MODEL.
Conversation text, recent context and robot status are sent to the model;
camera images and the gateway control token are not sent. The API uses
store=false. Do not enter secrets in the chat itself.

Examples to evaluate with a configured model:
- Take it a little easier.
- A little more. (after the previous slowdown)
- Actually, go right at the next junction.
- Back to the speed before that.
- Why are we stopped?
- Stop.

The model proposes one validated high-level intent. Node acknowledgment determines
whether the interface reports acceptance. Ambiguous or compound requests should
produce clarification rather than silently discarding part of the request.
The desktop sends heartbeats while open; losing the connection stops the bot.
Reconnection requires explicit Continue. Pressing STOP bypasses model latency. Direct stop and model-request cancellation
were tested. Actual language understanding still needs API-backed evaluation.

Starting position is a physical assertion: check the box only after placing the
bot after the first route junction and pointing toward the second. Setting a
route keeps a manual stop active. Junction calibration is a separate ROS setting.
The current map route is an assumed example; it is not automatic localization.

See docs/COMMAND_INTERFACE.md and docs/TESTING.md for the robot-side interface
and the distinction between software tests and physical track validation.

API implementation references:
- https://developers.openai.com/api/docs/guides/structured-outputs
- https://developers.openai.com/api/docs/models/gpt-5.4-mini

A small local-model alternative was evaluated but did not meet the context tests.
It is not the default. See docs/LOCAL_MODEL_EXPERIMENT.md for the measured failures.
The View route map button shows the proposed route using A-E labels.
