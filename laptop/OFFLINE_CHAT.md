# Offline request interpreter

Open **Start-OfflineChat.cmd** on Windows. This is a separate application from
the existing driving companion. It needs Python 3 with Tk; the launcher first
uses this laptop's bundled Python, then falls back to `py -3`.

The window interprets text only. It does not connect to a robot, call an API,
publish commands, run timers, measure distance, or simulate movement. It can be
used while duck2 is off. Closing it simply closes the window.

## Try these conversations

- `Slow down a little` → `A little more`
- `Take the next right` → `Actually left` → `The other direction`
- `Stop after two seconds` → `Make that three seconds`
- `Stop for five seconds` (a pause, distinct from a delayed stop)
- `Stop after thirty centimetres`
- `Stop at the next red line`
- `Interrupt all movement`
- `Reverse 20 centimetres at 10 centimetres per second`
- `Reverse for two seconds, slowly`
- `Reverse` → `twenty centimetres` → `at 10 cm/s`
- `Stop after five` → `seconds`
- `Hello`, `Thanks`, `Help`, `What is the robot doing?`

Replies describe the meaning, never claim successful robot actions. The JSON
panel exposes `category`, `parameters`, `needs_clarification`, `missing_fields`,
`reply`, and `interpretation_only: true`. Distances are metres and durations are
seconds. Speed retains its unit (`cm/s`, `m/s`, `%`, etc.) or qualitative wording;
percentages are not converted into physical speeds. Parameters are descriptions,
not wheel values or commands accepted by the existing gateway.

## Deliberate limits

This is a curated English parser with explicit memory, not an LLM. It supports
common numeric words, decimals, a bounded collection of paraphrases, and one
action per message. A reverse action may include its distance/duration and speed.
Unknown wording, missing units, and compound requests ask for clarification.
Negation and hypothetical/quoted requests are not positive movement requests.

`Stop until the checkpoint` needs clarification: stopping at a checkpoint and
waiting for an event are different meanings. Undo and retracing a turn are
deferred. Reverse capability and live robot state cannot be verified here.
Resume/continue and unrelated general conversation are outside this version's
movement vocabulary.

Memory is local to the open window and never written to disk. It represents the
last **discussed** request, not an executed action. A new request replaces that
context; unsupported, negated, hypothetical or unrecognized requests clear it.
Greetings and thanks preserve context. Use **Clear conversation** to reset it.
No simulated or conversational state is transferred to the driving companion.

## Run checks

From the repository root:

```text
python3 tests/test_offline_interpreter.py
python3 tests/test_chat.py
```

On Windows, using Python with Tk:

```text
py -3 tests/test_offline_chat_ui.py
```

The UI test constructs the actual window, interprets messages, clears it, and
closes it while network and child-process creation are blocked. It also blocks
imports of the existing connected companion and ROS. The interpreter tests
check both language behavior and the small offline-only import dependencies.

## Obstacle language

Try "duck", "obstacle", "yellow object blocking the path", "something in the way",
"there is a duck ahead", or "the road is blocked". These are user reports, never
camera-verified observations. Follow with "go around it", or say "avoid the duck"
or "bypass the obstacle". The interpretation describes a left pass, right-lane
return and retention of the prior route. It does not send anything to the robot.

Obstacle requests have no executable preview mapping and remove stale recordable
drafts. The controller prototype is separate and disabled pending calibration;
see [DUCK_AVOIDANCE.md](../docs/DUCK_AVOIDANCE.md).
