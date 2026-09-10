# Local language-model experiment

Qwen2.5-1.5B-Instruct Q4_K_M was evaluated through llama.cpp on this Windows
laptop as a possible way to avoid requiring an API key.

Sources:
- https://huggingface.co/Qwen/Qwen2.5-1.5B-Instruct-GGUF
- https://github.com/ggml-org/llama.cpp/blob/master/tools/server/README.md

Runtime: llama.cpp build b10809. The downloaded runtime ZIP and model weights
were verified against their published SHA-256 values before use.
The model server listened only on 127.0.0.1 and never connected to the robot.

Fourteen cases covered paraphrases, relative speed follow-ups, turn corrections,
speed restoration, questions, stopping/resuming, ambiguous commands, compound
requests, negation, safety-bypass requests and unsupported timing.

Initial instructions passed 6/14 cases. Two refined instruction formats each
passed 8/14. Failures included core user requirements such as interpreting an
opposite turn and restoring an earlier speed. At that historical stage the older connected app retained an API-backed
interpreter. The current `duck2_companion.py` instead uses `live_chat.py`, a
local deterministic grammar with no API key, model weights or training step.

The final detailed results are in local-chat-evaluation.json. They are failure
evidence, not a passing quality certificate. The experimental LocalInterpreter
is available for controlled evaluation but is not selected by the desktop UI.
The temporary model server was stopped and the large weights file removed.

The exercise also exposed useful application-level checks: contradictory
extra action fields are now rejected, and status replies use fresh robot data
rather than trusting model prose about whether the robot is moving.
