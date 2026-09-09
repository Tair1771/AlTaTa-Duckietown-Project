"""Offline route/chat state for the combined companion; no robot transport."""

from dataclasses import asdict, dataclass, field
import re

from offline_interpreter import OfflineInterpreter
from route_planner import MAP_ID, plan_route


@dataclass
class CompanionReply:
    category: str
    explanation: str
    action: str = None
    parameters: dict = field(default_factory=dict)
    needs_clarification: bool = False
    offline_only: bool = True

    def to_dict(self):
        return asdict(self)


class OfflineCompanionSession:
    """Own a route draft and contextual offline interpretation state."""

    def __init__(self):
        self.interpreter = OfflineInterpreter()
        self.start_approach = None
        self.destination_approach = None
        self.plan = None
        self.draft_valid = False
        self.history = []

    def reset(self):
        self.interpreter.reset()
        self.start_approach = self.destination_approach = self.plan = None
        self.draft_valid = False
        self.history = []

    def _reply(self, category, explanation, **kwargs):
        result = CompanionReply(category, explanation, **kwargs)
        self.history.append(result.to_dict())
        self.history = self.history[-20:]
        return result

    def _replan(self, first_turn=None):
        if self.start_approach and self.destination_approach:
            self.plan = plan_route(self.start_approach, self.destination_approach,
                                   required_first_turn=first_turn)
            self.draft_valid = True
        else:
            self.plan = None
            self.draft_valid = False

    def select_start(self, approach):
        old = self.start_approach
        self.start_approach = approach
        try:
            self._replan()
        except ValueError:
            self.start_approach = old
            self._replan()
            raise
        return self._reply("route_selection",
                           "I understood the starting lane as %s." % approach)

    def select_destination(self, approach):
        old = self.destination_approach
        self.destination_approach = approach
        try:
            self._replan()
        except ValueError:
            self.destination_approach = old
            self._replan()
            raise
        if self.plan:
            text = ("I understood destination %s. The shortest route crosses %d junction%s."
                    % (approach, self.plan.junction_count,
                       "" if self.plan.junction_count == 1 else "s"))
        else:
            text = "I understood destination %s. Select the starting lane next." % approach
        return self._reply("route_selection", text)

    @property
    def route_payload(self):
        if not self.plan or not self.draft_valid:
            return None
        payload = self.plan.to_dict()
        payload.update(action="set_route", position_confirmed=False,
                       execution_enabled=False)
        return payload

    @staticmethod
    def _approach_from_text(text, kind):
        letter = r"([A-E])"
        if kind == "destination":
            patterns = [
                (r"(?:destination|red line|go|drive|route|head)(?: to| at)?\s+" + letter
                 + r"\s+from\s+" + letter, True),
                (r"(?:destination|red line)(?: is|:)?\s+" + letter
                 + r"\s*->\s*" + letter, False),
            ]
        else:
            patterns = [
                (r"(?:start(?:ing)?|starting lane|currently)(?: on| at|:)?\s+" + letter
                 + r"\s*(?:to|->|toward|towards)\s*" + letter, False),
                (r"between\s+" + letter + r"\s+and\s+" + letter
                 + r"\s+(?:facing|toward|towards)\s+" + letter, False),
            ]
        for pattern, reverse in patterns:
            match = re.search(pattern, text, re.IGNORECASE)
            if not match:
                continue
            groups = tuple(value.upper() for value in match.groups())
            if kind == "destination":
                # Natural wording is "junction C from B": travel B -> C.
                return ((groups[1] + "->" + groups[0]) if reverse
                        else (groups[0] + "->" + groups[1]))
            if len(groups) == 2:
                return groups[0] + "->" + groups[1]
            # "between A and B facing B" means A -> B.
            first, second, facing = groups
            return (first + "->" + second) if facing == second else second + "->" + first
        return None

    def chat(self, text):
        message = text.strip()
        lowered = message.lower().rstrip(".!?")
        if not message:
            return self._reply("clarification", "Please enter a message.",
                               needs_clarification=True)
        if lowered in ("cancel", "cancel route", "clear route", "never mind", "nevermind"):
            self.destination_approach = self.plan = None
            self.draft_valid = False
            return self._reply("route_cancelled",
                               "I cleared the unexecuted route draft. No robot movement was cancelled.")
        if any(phrase in lowered for phrase in
               ("where are we going", "what is the route", "show route", "route status")):
            if not self.plan:
                return self._reply("route_status", "No route draft is complete yet.")
            return self._reply("route_status",
                "The draft goes %s to destination %s and crosses %d junction%s."
                % (" -> ".join(self.plan.route), self.plan.destination_approach,
                   self.plan.junction_count,
                   "" if self.plan.junction_count == 1 else "s"))

        start = self._approach_from_text(message, "start")
        destination = self._approach_from_text(message, "destination")
        try:
            if start:
                return self.select_start(start)
            if destination:
                return self.select_destination(destination)
        except ValueError as error:
            self.draft_valid = False
            return self._reply("clarification", str(error), needs_clarification=True)

        direct_turn = re.search(r"\b(left|right|straight)\b", lowered)
        if (direct_turn and any(word in lowered for word in ("turn", "take", "actually"))
                and not any(word in lowered for word in ("not ", "don't", "do not", "if "))):
            direction = direct_turn.group(1)
            if not self.plan:
                self.draft_valid = False
                return self._reply("clarification",
                    "Select a starting lane and destination before changing the next turn.",
                    needs_clarification=True)
            try:
                self._replan(direction)
            except ValueError as error:
                self.draft_valid = False
                return self._reply("clarification", str(error), needs_clarification=True)
            return self._reply("route_updated",
                "I understood the next turn as %s and recalculated the remaining route to %s."
                % (direction, self.destination_approach),
                action="turn", parameters={"value": direction})

        interpretation = self.interpreter.interpret(message)
        if interpretation.needs_clarification or interpretation.category in (
                "negated", "hypothetical", "unsupported"):
            self.draft_valid = False
            return self._reply(interpretation.category, interpretation.reply,
                               needs_clarification=True)
        if interpretation.category == "turn":
            direction = interpretation.parameters.get("direction")
            if not self.plan:
                self.draft_valid = False
                return self._reply("clarification",
                    "Select a starting lane and destination before changing the next turn.",
                    needs_clarification=True)
            try:
                self._replan(direction)
            except ValueError as error:
                self.draft_valid = False
                return self._reply("clarification", str(error), needs_clarification=True)
            return self._reply("route_updated",
                "I understood the next turn as %s and recalculated the remaining route to %s."
                % (direction, self.destination_approach),
                action="turn", parameters={"value": direction})
        if interpretation.category == "stop":
            self.draft_valid = False
            return self._reply("command_preview", interpretation.reply,
                               action="stop", parameters=interpretation.parameters)
        if interpretation.category in ("speed_change", "speed_setting", "interrupt",
                                       "reverse", "obstacle"):
            # These retain the interpreter's exact limitations; no route command is substituted.
            self.draft_valid = False
            return self._reply("command_preview", interpretation.reply,
                               parameters=interpretation.parameters,
                               needs_clarification=interpretation.needs_clarification)
        return self._reply("conversation", interpretation.reply)
