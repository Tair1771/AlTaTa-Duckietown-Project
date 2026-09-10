# Windows companion

Open `Start-Duck2Companion.cmd` for the current map, camera and live-chat app.
Prepare its backend first: [STARTUP](../docs/STARTUP.md),
[operation](../docs/USAGE.md), [chat commands](../docs/LIVE_CHAT.md).
Shortcuts prefer this laptop's bundled Python, then the standard `py -3` launcher.
Install `requirements-desktop.txt` in that interpreter for camera resizing.

| Retained shortcut | Purpose |
| --- | --- |
| `Start-Duck2Companion.cmd` | Normal app; explicitly connect and start viewing |
| `Start-Duck2-JunctionChatCheck.cmd` | Current A → E chat-override scenario; empty chat input |
| `Start-Duck2-PauseCheck.cmd` | Pause check on A → E ending at the next red line |
| `Start-BenchCompanion.cmd` | Isolated simulation app; run bench backend first |

None presses physical Start automatically. The scenario shortcuts connect and
view the camera. The ordinary app can also plan while disconnected.

`duck2_companion.py` is the entry point; `live_chat.py` implements the local
command grammar and session queue. `route_planner.py` shares the directed map
with the robot. `camera_client.py`, `route_control.py`, `companion_core.py` and
parts of `chat_core.py` support the current app. Do not remove `chat_core.py`
because its name resembles the old chatbot: the companion imports its transport.

## Retained legacy tools

Three obsolete duplicate `.cmd` shortcuts were removed after a recovery backup.
Their implementation and tests remain available without cluttering the launcher list:

```powershell
py -3 laptop/offline_chat.py
py -3 laptop/command_preview_chat.py
py -3 laptop/duck2_chat.py
```

The first two are offline draft tools. The third is a historical connected app,
not the current live-chat workflow. See [legacy details](LEGACY_APPS.md),
[offline interpreter](OFFLINE_CHAT.md) and [preview](COMMAND_PREVIEW.md).
