# Offline command preview

On Windows, install Python 3 with Tkinter and run `Start-CommandPreview.cmd`, or
run `python command_preview_chat.py` from this folder. Only standard Python
libraries are required. The launcher also supports the bundled local Python.

This is a separate app. `Start-OfflineChat.cmd` still opens the unchanged
interpretation-only app. Neither app needs an API key or a robot connection.

Try: **take the next right → actually left → Record preview**. The history
contains a local test record for a left-turn request. No command is delivered,
no movement is simulated, and controller acceptance is not requested.

Basic stop, speed up/down, and next left/right/straight map to the existing
controller vocabulary. Speed steps mean multiplying the current speed scale by
1.2 or 0.8, subject to the controller's limits; they are not measured velocities.
Amounts such as “a little” or “by 10 percent” first ask whether to substitute the
standard step. Reply **use the standard step** or **cancel**. A missing number
unit must be resolved before that substitution can be offered.

Turns require a configured route, an available exit and controller acceptance.
Calibration and physical readiness are unknown. Timed/distance/checkpoint stops,
pauses, reverse, undo, absolute speed and interruption with cancellation have no
execution mapping here. An interruption is never silently reduced to plain stop.

Chatting creates or replaces one draft. Recording consumes that draft; another
record needs a new request or correction. Corrections change future drafts and
never rewrite history. Cancel clears a draft and discussion context but retains
records. Greetings and thanks preserve a draft without creating one. New
ambiguous, negated or unsupported requests clear the recordable draft. Clear
conversation resets context, draft and records; closing discards all memory.

`Preview` contains `status`, `action`, `parameters`, `explanation`,
`prerequisites`, and `offline_only`. Status is `preview_available`,
`clarification_needed`, `unsupported_request`, or `conversation_only`.
There are no delivery IDs, timestamps or heartbeats. `FakeReceiver` checks the
action/parameter allowlist and stores independent copies only in memory.

Run `python -X utf8 tests/test_command_preview.py` from the repository root. Windows
also exercises the actual Tk window with network access, child processes and
connected application imports blocked. Existing tests remain available as
`tests/test_offline_interpreter.py` and `tests/test_offline_chat_ui.py`.
Use `-X utf8` for the existing interpreter tests on Windows as well, because
they read source files containing Unicode punctuation.

Verification: 35 chatbot test methods passed on Windows (10 command-preview,
13 existing interpreter, one existing interpretation-window, and 11 existing
companion tests). The new and existing window checks blocked network access.

These tests validate interpretation and translation, not live command delivery,
controller acceptance or physical completion. Existing robot and gateway code
is unchanged by this feature.
