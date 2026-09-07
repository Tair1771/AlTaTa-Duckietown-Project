"""Offline translation and local test records. No transport or robot state."""
import copy
from dataclasses import asdict, dataclass, field

from offline_interpreter import OfflineInterpreter, normalize


@dataclass
class Preview:
    status: str
    action: str = None
    parameters: dict = field(default_factory=dict)
    explanation: str = ""
    prerequisites: list = field(default_factory=list)
    offline_only: bool = True

    def to_dict(self):
        return asdict(self)


def translate(result):
    """Map complete interpretations to a bounded, non-deliverable preview."""
    if result.needs_clarification:
        return Preview("clarification_needed", explanation=result.reply)
    p = result.parameters
    if result.category == "stop" and p.get("condition") == "immediate":
        return Preview("preview_available", "stop", explanation="I understood an immediate stop request.")
    if result.category == "speed_change":
        if p.get("direction") not in ("increase", "decrease"):
            return Preview("clarification_needed", explanation="Which speed direction do you mean?")
        if "amount" in p or "degree" in p:
            return Preview("clarification_needed", explanation=(
                result.reply + " The available command uses a fixed relative step: increase by 20% "
                "or decrease by 20% of the current speed scale. It does not set a measured velocity. "
                "Use that standard step instead? Reply 'use the standard step' or 'cancel'."))
        action = "speed_up" if p["direction"] == "increase" else "slow_down"
        return Preview("preview_available", action, explanation=(
            "I understood a standard speed step: " + ("increase" if action == "speed_up" else "decrease") +
            " by 20% of the current speed scale, not a measured velocity."),
            prerequisites=["Controller must accept the resulting speed scale within its limits.",
                           "Live state and physical readiness are unknown."])
    if result.category == "turn" and p.get("direction") in ("left", "right", "straight"):
        return Preview("preview_available", "turn", {"value": p["direction"]},
                       "I understood the next turn: " + p["direction"] + ".",
                       ["A route and starting position must be configured.",
                        "The requested exit must be available and accepted by the controller.",
                        "Physical readiness and junction calibration are unknown."])
    if result.category in ("stop", "interrupt", "reverse", "unsupported", "speed_setting"):
        return Preview("unsupported_request", explanation=(result.reply +
            " This request has no execution mapping in this preview app. No simpler command was substituted."))
    if result.category in ("negated", "hypothetical", "clarification"):
        return Preview("clarification_needed", explanation=result.reply)
    return Preview("conversation_only", explanation=result.reply)


class FakeReceiver:
    """Records only validated previews in memory; cannot deliver them anywhere."""
    def __init__(self):
        self._records = []

    @property
    def records(self):
        return copy.deepcopy(self._records)

    def record(self, preview):
        if (not isinstance(preview, Preview) or preview.status != "preview_available"
                or preview.offline_only is not True or not isinstance(preview.parameters, dict)):
            raise ValueError("Only an available offline preview can be recorded")
        valid = ((preview.action in ("stop", "speed_up", "slow_down") and preview.parameters == {})
                 or (preview.action == "turn" and set(preview.parameters) == {"value"}
                     and preview.parameters["value"] in ("left", "right", "straight")))
        if not valid:
            raise ValueError("Unsupported preview action or parameters")
        record = {"record_number": len(self._records)+1, "kind": "local_test_record",
                  "preview": preview.to_dict(), "controller_acceptance": "not_requested",
                  "physical_completion": "unknown"}
        self._records.append(copy.deepcopy(record))
        return copy.deepcopy(record)

    def clear(self):
        self._records.clear()


class PreviewSession:
    def __init__(self):
        self.interpreter = OfflineInterpreter()
        self.receiver = FakeReceiver()
        self._draft = None
        self._step_direction = None

    @property
    def draft(self):
        return copy.deepcopy(self._draft)

    def chat(self, text):
        message = normalize(text) if isinstance(text, str) else ""
        if message in ("a little faster", "a little slower"):
            text = "speed up a little" if message.endswith("faster") else "slow down a little"
        if message in ("cancel", "cancel that", "forget that", "reset conversation"):
            self.interpreter.reset()
            self._draft = self._step_direction = None
            return Preview("conversation_only", explanation=
                           "The unrecorded draft is cleared. No robot movement was cancelled.")
        if self._step_direction and message in ("yes", "yes please", "use the standard step", "standard step"):
            direction = self._step_direction
            result = self.interpreter.interpret("speed up" if direction == "increase" else "slow down")
        else:
            result = self.interpreter.interpret(text)
        preview = translate(result)
        if preview.status != "conversation_only":
            self._draft = copy.deepcopy(preview) if preview.status == "preview_available" else None
            self._step_direction = (result.parameters.get("direction")
                                    if result.category == "speed_change" and not result.needs_clarification
                                    and preview.status == "clarification_needed" else None)
        return preview

    def record(self):
        record = self.receiver.record(self._draft)
        self._draft = None
        return record

    def clear(self):
        self.interpreter.reset()
        self._draft = self._step_direction = None
        self.receiver.clear()
