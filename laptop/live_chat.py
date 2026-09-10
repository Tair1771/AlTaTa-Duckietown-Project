"""Local live-chat interpretation and one-at-a-time junction transactions.

No model API, wheel commands, or guessed localization. The caller serializes
methods with the same lock used by Start and Stop. Heartbeats remain separate.
"""
import re
import uuid
from dataclasses import dataclass

from route_planner import MAP_PORTS, junction_turn, parse_approach, approach_id


@dataclass(frozen=True)
class Intent:
    action: str
    value: object = None
    append: bool = False


def interpret_live(text):
    text = re.sub(r"(?<!\d)\.|\.(?!\d)|[!?,;]+", " ", text.lower().strip())
    text = re.sub(r"\s+", " ", text).strip()
    if not text or len(text) > 500:
        return Intent("clarify")
    # Negated, hypothetical and unsupported requests must never become motion.
    if re.search(r"\b(don't|dont|not|never|unless|if|reverse|backward|backwards|overtake|obstacle)\b", text):
        return Intent("clarify")
    text = re.sub(r"^(?:please |can you |could you |will you |i want (?:you |it |the bot |duck2 )?to |i would like to |let's |lets )+", "", text)
    text = re.sub(r"\s+please$", "", text)
    if re.fullmatch(r"(?:(?:quit|exit)(?: (?:the |this )?run)?|(?:end|stop) (?:the |this )?run)(?: now)?", text):
        return Intent("end")
    if re.fullmatch(r"(?:continue|resume|keep going|go on|cancel (?:the )?pause)(?: now)?", text):
        return Intent("resume")
    pause = re.fullmatch(r"(?:pause|stop|wait|hold on|hold)(?: (?:the )?(?:bot|robot|duck2|run))?(?: (?:for )?(\d+(?:\.\d+)?|one|two|three|four|five|six|seven|eight|nine|ten|twenty|thirty)\s*(?:seconds?|secs?|s))?(?: now)?", text)
    if pause:
        value = pause.group(1)
        words = dict(zip("one two three four five six seven eight nine ten twenty thirty".split(),
                         [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 30]))
        seconds = None if value is None else words[value] if value in words else float(value)
        if seconds is not None and not 0 < seconds <= 3600:
            return Intent("clarify")
        return Intent("pause", seconds)
    if text in ("status", "where are we", "where are you", "what is next", "what's next", "show queue", "show route"):
        return Intent("status")
    if text in ("clear queue", "clear turns", "cancel route", "cancel turns"):
        return Intent("turns", ())
    profiles = {"slow": "slow", "slowest": "slow", "normal": "normal", "medium": "normal",
                "middle": "normal", "default": "normal", "fast": "fast", "fastest": "fast"}
    speed = re.fullmatch(r"(?:(?:set |use |go )?(?:(?:the )?speed (?:to )?)?)?(slow|slowest|normal|medium|middle|default|fast|fastest)(?: (?:speed|profile))?", text)
    if speed:
        return Intent("profile", profiles[speed.group(1)])
    if text in ("speed up", "faster", "go faster", "increase speed"):
        return Intent("profile", "up")
    if text in ("slow down", "slower", "go slower", "decrease speed"):
        return Intent("profile", "down")
    append = bool(re.search(r"\b(after that(?: one)?|afterwards|append|add|then|also)\b", text))
    # 'left then right' replaces the queue; 'left after that one' appends.
    if re.search(r"\b(left|right|straight)\s+(?:and )?then\s+", text):
        append = False
    turns = re.findall(r"\b(left|right|straight)\b", text)
    rest = re.sub(r"\b(left|right|straight)\b", " ", text)
    rest = re.sub(r"\b(the|next|turn|turns|should|be|take|go|at|on|junction|intersection|red|line|light|and|then|after|that|one|afterwards|append|add|also|actually|make|it|to|ahead)\b", " ", rest)
    if turns and not rest.strip() and len(turns) <= 30:
        return Intent("turns", tuple(turns), append)
    # Ambiguous stop wording stays a pause; explicit end-run wording above
    # is the only textual action that clears the run.
    if re.search(r"\b(stop|pause)\b", text) and not turns and not re.search(r"\d|\b(for|seconds?|minutes?)\b", text):
        return Intent("pause")
    return Intent("clarify")


def follow_turn(approach, turn):
    previous, junction = parse_approach(approach)
    choices = [node for node in MAP_PORTS[junction].values()
               if node != previous and junction_turn(previous, junction, node) == turn]
    if len(choices) != 1:
        raise ValueError("No %s exit at %s when approaching from %s. Queue unchanged." %
                         (turn, junction, previous))
    return approach_id(junction, choices[0])


class LiveChatSession:
    def __init__(self, start, turns=(), run_id=None):
        parse_approach(start)
        self.run_id = run_id or str(uuid.uuid4())
        self.approach = start
        self.index = 1
        self.queue = []
        self.inflight = None
        self.active = True
        self.confirmed = False
        self.last_status = None
        self.notices = []
        self.asked_index = None
        self.replace_turns(turns)

    def replace_turns(self, turns, append=False):
        if not self.active:
            raise ValueError("The run ended. Confirm placement and Start a new run.")
        if self.inflight and not self.inflight["accepted"]:
            raise ValueError("The last instruction has an unknown outcome; wait for fresh robot status.")
        proposed = (list(self.queue) if append else []) + list(turns)
        if len(proposed) > 30:
            raise ValueError("Use at most 30 future turns")
        location = self.inflight["outgoing"] if self.inflight else self.approach
        for turn in proposed:
            location = follow_turn(location, turn)
        self.queue = proposed
        return "Future turns: %s. Applied only at red-line stops." % (" → ".join(self.queue) or "none")

    def cancel(self):
        self.active = False
        self.queue.clear()
        self.inflight = None

    def observe(self, status):
        self.last_status = status
        live = status.get("live_session") or {}
        if not self.active:
            return
        if live.get("run_id") != self.run_id or live.get("version") != 1:
            raise ValueError("Robot session differs from this laptop's queue; confirm placement and Start again.")
        self.confirmed = True
        if not live.get("active"):
            # Robot-side completion can arrive before the next queue poll.
            # Accept only the outgoing edge of our own completed instruction.
            result = status.get("junction_last_result") or {}
            if (self.inflight
                    and live.get("instruction_id") == self.inflight["id"]
                    and status.get("route_index") == self.index + 1
                    and status.get("current_approach") == self.inflight["outgoing"]
                    and result.get("route_index") == self.index + 1
                    and result.get("turn") == self.inflight["turn"]
                    and result.get("outcome") in ("reacquired", "reacquired_at_next_red")):
                self.approach = self.inflight["outgoing"]
                self.index += 1
            self.cancel()
            self.notices.append(live.get("end_reason") or "Run ended on the robot.")
            return
        index = status.get("route_index")
        if isinstance(index, bool) or not isinstance(index, int):
            raise ValueError("Robot did not report its junction progress")
        incoming = status.get("current_approach")
        pending = self.inflight
        if pending and live.get("instruction_id") == pending["id"]:
            self._accept_instruction()
        if index == self.index:
            if incoming != self.approach:
                raise ValueError("Robot lane disagrees with confirmed turn history")
        elif index == self.index + 1 and pending and pending["accepted"]:
            result = status.get("junction_last_result") or {}
            if (result.get("route_index") != index or result.get("turn") != pending["turn"]
                    or result.get("outcome") not in ("reacquired", "reacquired_at_next_red")
                    or incoming != pending["outgoing"]):
                raise ValueError("Turn completion is not confirmed; queue delivery suspended")
            self.approach, self.index = incoming, index
            self.inflight = None
            self.notices.append("Completed %s; reported lane is %s." % (pending["turn"], incoming))
        else:
            raise ValueError("Unexpected junction progress; queue delivery suspended")
        if status.get("state") == "red_stop" and not self.queue and not self.inflight:
            if self.asked_index != index:
                self.asked_index = index
                seconds = live.get("red_wait_seconds", 30)
                self.notices.append("Where should I go next? The run ends %g seconds after arriving at this red line." % seconds)

    def _accept_instruction(self):
        if not self.inflight["accepted"]:
            if not self.queue or self.queue[0] != self.inflight["turn"]:
                raise ValueError("Pending queue changed unexpectedly")
            self.queue.pop(0)
            self.inflight["accepted"] = True

    def service(self, transport, status):
        """Use fresh status, reconcile any lost acknowledgment, deliver once."""
        self.observe(status)
        if not self.active:
            return
        if self.inflight and self.inflight["accepted"]:
            return
        if not status.get("junction_instruction_ready") or not self.queue:
            return
        if self.inflight is None:
            self.inflight = {"id": str(uuid.uuid4()), "turn": self.queue[0],
                             "outgoing": follow_turn(self.approach, self.queue[0]),
                             "accepted": False, "attempts": 0,
                             "epoch": status["control_epoch"]}
        pending = self.inflight
        if pending["attempts"] >= 3:
            raise ValueError("Junction acknowledgment unavailable. Stop and reconnect; no further instruction was sent.")
        pending["attempts"] += 1
        ack = transport.send("junction_instruction", command_id=pending["id"],
                             value=pending["turn"], approach=self.approach,
                             expected_route_index=self.index, run_id=self.run_id,
                             expected_control_epoch=pending["epoch"])
        if ack.get("accepted") is not True:
            self.inflight = None
            raise ValueError(str(ack.get("reason", "Junction instruction rejected")))
        self._accept_instruction()
        self.notices.append("%s accepted at %s; waiting for outgoing-lane confirmation." %
                            (pending["turn"].title(), self.approach))

    def execute(self, intent, transport, status):
        self.observe(status)
        if not self.active:
            raise ValueError("Run ended. Confirm placement and Start again.")
        if intent.action == "turns":
            return self.replace_turns(intent.value, intent.append)
        if intent.action == "status":
            return self.summary()
        if intent.action not in ("pause", "resume", "profile"):
            raise ValueError("Use a turn, pause, continue, speed profile, or Stop button.")
        fields = {"run_id": self.run_id, "expected_control_epoch": status["control_epoch"]}
        action = intent.action
        if action == "pause":
            fields["seconds"] = intent.value
        if action == "profile":
            action = "straight_profile"
            order = ("slow", "normal", "fast")
            current = (status.get("live_session") or {}).get("profile", "normal")
            choice = intent.value
            if choice in ("up", "down"):
                choice = order[max(0, min(2, order.index(current) + (1 if choice == "up" else -1)))]
            fields["value"] = choice
        ack = transport.send(action, **fields)
        if ack.get("accepted") is not True:
            raise ValueError(str(ack.get("reason", "Command rejected")))
        if action == "pause":
            return ("Stopping now. Queue kept. "
                    + ("Resume automatically after %s seconds if the camera and connection stay healthy. " % intent.value
                       if intent.value is not None else "Say continue within 30 seconds of the pause. ")
                    + "STOP DUCK2 or 'quit' ends the run immediately.")
        if action == "resume":
            return "Continue accepted. Red stops still require a valid turn."
        return "%s profile selected for confirmed straight roads only; curve and turn speeds are unchanged." % fields["value"].title()

    def summary(self):
        live = (self.last_status or {}).get("live_session") or {}
        phase = "ended" if not self.active else "active"
        if self.active and live.get("paused"):
            remaining = live.get("pause_remaining")
            phase = "paused" + (" (%.1fs remaining)" % remaining if remaining is not None else " (30s Continue limit)")
        elif self.active and live.get("pause_pending"):
            phase = "pause pending — waiting for a straight"
        elif self.active and (self.last_status or {}).get("state") == "red_stop":
            remaining = live.get("wait_remaining")
            phase = "at red line" + (" (%.1fs remaining)" % remaining if remaining is not None else "")
        return "%s • %s straight profile\nReported lane: %s%s\nFuture turns: %s" % (
            phase, live.get("profile", "normal"),
            self.approach, " (crossing toward %s)" % self.inflight["outgoing"] if self.inflight else "",
            " → ".join(self.queue) or "none")

    def take_notices(self):
        notices, self.notices = self.notices, []
        return notices
