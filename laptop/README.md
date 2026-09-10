# Windows companion

Open **Start-Duck2Companion.cmd** or run `py -3 laptop/duck2_companion.py` from
the repository root. Install `requirements-desktop.txt` in the same interpreter;
Tcl/Tk is required and Pillow supplies camera rendering.

The app contains Map & route and Camera & status, with Live chat beside the
camera. Opening it does not connect automatically, start movement, prefill
commands or guess robot position. No API key is required.

[Setup](../docs/STARTUP.md) · [Operation](../docs/USAGE.md) · [Chat](../docs/LIVE_CHAT.md)

Keep these modules together: `companion_core.py` owns local map selection,
`route_planner.py` runs directed A*, `live_chat.py` handles the command grammar
and turn queue, `route_control.py` performs confirmed Start, `chat_core.py`
provides HTTP transport, and `camera_client.py` retrieves read-only previews.
