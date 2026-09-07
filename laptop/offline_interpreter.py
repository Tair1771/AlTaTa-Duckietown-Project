"""Bounded English request interpretation. This module has no robot or network I/O.

Conversation memory records discussed requests, never executed movements.
The vocabulary is deliberately finite: unmatched text asks for clarification.
"""
import copy
import re
from dataclasses import asdict, dataclass, field


@dataclass
class Interpretation:
    category: str
    parameters: dict = field(default_factory=dict)
    needs_clarification: bool = False
    reply: str = ""
    missing_fields: list = field(default_factory=list)
    interpretation_only: bool = True

    def to_dict(self):
        return asdict(self)


ONES = dict(zip("zero one two three four five six seven eight nine ten eleven twelve thirteen fourteen fifteen sixteen seventeen eighteen nineteen".split(), range(20)))
TENS = dict(zip("twenty thirty forty fifty sixty seventy eighty ninety".split(), range(20, 100, 10)))
NUMBER_WORDS = set(ONES) | set(TENS) | {"hundred", "thousand", "point"}
NUMBER = r"[+-]?(?:\d+(?:\.\d+)?|\.\d+)"
DISTANCE = {
    "m": 1, "meter": 1, "meters": 1, "metre": 1, "metres": 1,
    "cm": .01, "centimeter": .01, "centimeters": .01, "centimetre": .01, "centimetres": .01,
    "mm": .001, "millimeter": .001, "millimeters": .001, "millimetre": .001, "millimetres": .001,
    "ft": .3048, "foot": .3048, "feet": .3048, "inch": .0254, "inches": .0254,
    "km": 1000, "kilometer": 1000, "kilometers": 1000, "kilometre": 1000, "kilometres": 1000,
}
TIME = {"s": 1, "sec": 1, "secs": 1, "second": 1, "seconds": 1,
        "min": 60, "mins": 60, "minute": 60, "minutes": 60,
        "ms": .001, "millisecond": .001, "milliseconds": .001,
        "h": 3600, "hour": 3600, "hours": 3600}
SPEED_UNITS = {"m/s": "m/s", "cm/s": "cm/s", "mm/s": "mm/s", "km/h": "km/h", "mph": "mph",
               "%": "%", "percent": "%", "per cent": "%"}
for _unit, _canonical in [("meter", "m/s"), ("metre", "m/s"), ("centimeter", "cm/s"),
                          ("centimetre", "cm/s"), ("millimeter", "mm/s"), ("millimetre", "mm/s")]:
    for _plural in ("", "s"):
        SPEED_UNITS[_unit + _plural + " per second"] = _canonical
SPEED_UNITS.update({"kilometres per hour": "km/h", "kilometers per hour": "km/h", "miles per hour": "mph"})
CHECKPOINT = r"(?:the )?(?:next )?(?:red line|checkpoint)(?: before (?:the |a )?(?:next )?turn)?"
REQUEST_CATEGORIES = {"speed_change", "speed_setting", "turn", "stop", "interrupt", "reverse", "obstacle"}


def _number_words(match):
    words = match.group().replace("-", " ").split()
    if "point" in words:
        index = words.index("point")
        if not words[index+1:] or any(w not in ONES or ONES[w] > 9 for w in words[index+1:]):
            return match.group()
        integer = _number_words(re.match(r".*", " ".join(words[:index]))) if index else "0"
        if not integer.isdigit():
            return match.group()
        return integer + "." + "".join(str(ONES[w]) for w in words[index+1:])
    total = current = 0
    previous = None
    for word in words:
        if word == "and":
            continue
        if word in ONES or word in TENS:
            # Reject malformed sequences such as 'two three' rather than summing them.
            if previous in ONES or (previous in TENS and (word in TENS or ONES.get(word, 10) >= 10)):
                return match.group()
            current += ONES.get(word, TENS.get(word, 0))
        elif word == "hundred":
            if previous not in ONES or current < 1 or current > 9:
                return match.group()
            current *= 100
        elif word == "thousand":
            if not current or total:
                return match.group()
            total, current = current * 1000, 0
        else:
            return match.group()
        previous = word
    return str(total + current)


def normalize(text):
    text = text.lower().replace("’", "'").strip()
    # Preserve decimal points, signs, slashes and percentages.
    text = re.sub(r"[?!,;:]", " ", text).strip().rstrip(".! ")
    text = re.sub(r"\s+", " ", text)
    words = "|".join(sorted(NUMBER_WORDS, key=len, reverse=True))
    text = re.sub(r"\b(?:" + words + r")(?:(?:[ -]+| and )(?:" + words + r"))*\b", _number_words, text)
    text = re.sub(r"\b(?:a|an) (second|minute|hour|metre|meter|centimetre|centimeter)\b", r"1 \1", text)
    text = re.sub(r"^(?:please |(?:can|could|would) you (?:please )?|i (?:want|would like) you to )", "", text)
    text = re.sub(r" please$", "", text)
    return text.strip()


def quantity(text):
    """Return a normalized limit, an unresolved bare number, or None."""
    match = re.fullmatch(r"(" + NUMBER + r")\s*([a-z]+)?", text)
    if not match:
        return None
    value, unit = float(match[1]), match[2]
    if not 0 < value <= 1e9:
        return {"invalid": True}
    if unit in DISTANCE or unit in TIME:
        normalized = round(value * (DISTANCE.get(unit) or TIME[unit]), 9)
        if normalized <= 0:
            return {"invalid": True}
        return {"distance_m" if unit in DISTANCE else "duration_s": normalized}
    if unit is None:
        return {"unresolved_amount": value}
    return None


def speed(text):
    if text in ("slow", "slowly", "very slowly", "fast", "quickly", "at a slow speed", "at a low speed"):
        return {"description": text}
    match = re.fullmatch(r"(" + NUMBER + r")\s*(.*)", text)
    if not match:
        return None
    value, unit = float(match[1]), SPEED_UNITS.get(match[2])
    if not 0 < value <= 1e9:
        return {"invalid": True}
    if not match[2]:
        return {"value": value, "unit": None}
    return {"value": value, "unit": unit} if unit else None


def clarification(reply, category="clarification", parameters=None, missing=None):
    return Interpretation(category, parameters or {}, True, reply, missing or [])


def complete(category, parameters):
    """Describe an interpretation, never an execution or capability claim."""
    p = parameters
    if category == "reverse":
        if "unresolved_amount" in p:
            return clarification("What distance or duration units do you mean?", category, p, ["limit_unit"])
        if not ("distance_m" in p or "duration_s" in p):
            return clarification("What reverse distance or duration do you mean?", category, p, ["distance_or_duration"])
        if p.get("speed", {}).get("unit", "known") is None:
            return clarification("What speed units do you mean: percent, cm/s, or m/s?", category, p, ["speed_unit"])
    if category == "obstacle":
        if p.get("intent") == "avoid":
            description = "a request to pass the obstacle on the left, return to the right lane, and retain the previous route"
        else:
            description = "your report of an obstacle in the path"
        return Interpretation(category, p, reply="I understood: " + description +
                              ". Interpretation only; nothing was sent or executed. "
                              "I cannot see the robot camera or confirm a clear passing lane. "
                              "Experimental automatic passing needs physical calibration.")
    if category == "speed_change":
        description = ("increase" if p["direction"] == "increase" else "decrease") + " the speed"
        if "amount" in p:
            description += " by " + str(p["amount"]["value"]) + p["amount"]["unit"]
        elif p.get("degree"):
            description += " " + p["degree"]
    elif category == "speed_setting":
        description = "set the speed to " + _speed_text(p["speed"])
    elif category == "turn":
        description = "go " + p["direction"] + " at the next junction"
    elif category == "interrupt":
        description = "stop immediately and cancel all pending movement"
    elif category == "stop":
        condition = p["condition"]
        if condition == "immediate":
            description = "stop immediately"
        elif condition == "at_checkpoint":
            description = "stop upon reaching the next red-line checkpoint"
        elif condition == "pause":
            description = "pause for %g seconds" % p["duration_s"]
        elif condition == "after_duration":
            description = "stop after a delay of %g seconds" % p["duration_s"]
        else:
            description = "stop after travelling %g metres" % p["distance_m"]
    else:
        limit = ("%g metres" % p["distance_m"] if "distance_m" in p else "%g seconds" % p["duration_s"])
        description = "reverse for " + limit
        if "speed" in p:
            description += " at " + _speed_text(p["speed"])
    reply = "I understood: " + description + ". Interpretation only; nothing was sent or executed."
    if category == "reverse":
        reply += " The robot's ability to reverse has not been verified."
    return Interpretation(category, p, reply=reply)


def _speed_text(value):
    return value.get("description") or "%g %s" % (value["value"], value["unit"])


class OfflineInterpreter:
    """Per-window, in-memory conversation. No access to robot state or transport."""
    def __init__(self):
        self.last_request = None
        self.pending = None

    def reset(self):
        self.last_request = self.pending = None

    def interpret(self, text):
        if not isinstance(text, str) or not 1 <= len(text.strip()) <= 2000:
            return clarification("Enter a message of 1–2000 characters.")
        message = normalize(text)
        result = self._interpret(message)
        if result.category in REQUEST_CATEGORIES:
            if result.needs_clarification:
                self.pending = copy.deepcopy(result)
            else:
                self.last_request = copy.deepcopy(result)
                self.pending = None
        elif result.category in ("negated", "hypothetical", "unsupported", "clarification"):
            # Do not leave a rejected movement as a possible follow-up target.
            self.reset()
        return result

    def _interpret(self, message):
        if re.search(r"\b(?:don't|do not|not|never|cannot|can't|shouldn't|wouldn't|mustn't|no)\b", message):
            return Interpretation("negated", reply="I understood a negation, not a positive movement request. Nothing was sent or executed.")
        if re.search(r"\b(?:if|suppose|imagine|hypothetically|example|said|says|say|would happen)\b", message) or any(c in message for c in ('"', "“", "”")):
            return Interpretation("hypothetical", reply="That describes a hypothetical or quoted request. I have not treated it as a movement instruction.")
        if re.search(r"\b(?:undo|retrace|backtrack)\b", message) or re.search(r"\b(?:reverse|back) (?:to|through)\b", message):
            return Interpretation("unsupported", reply="Movement undo and reversing back through a previous turn are deferred. Nothing was sent or executed.")
        if message in ("hello", "hi", "hey", "good morning", "good evening"):
            return Interpretation("conversation", reply="Hello! Tell me a robot request and I'll explain its meaning. This chat cannot control a robot.")
        if message in ("thanks", "thank you", "thank you very much", "great", "okay", "ok"):
            return Interpretation("conversation", reply="You're welcome. We are only discussing requests; no robot actions are being performed.")
        if message in ("help", "what can you do", "what commands do you understand", "what can i say", "how does this work"):
            return Interpretation("help", reply="I can interpret speed changes, next turns, immediate or conditional stops, interruption, reverse requests with a distance or duration, and obstacle reports or avoidance requests. Try 'stop after two seconds'. I cannot execute requests or undo movements.")
        if re.fullmatch(r"(?:can (?:the (?:bot|robot)|duck2|it) (?:move backwards|reverse)|(?:what|how|where) (?:is|are) .+|status|are you moving|how fast .+)", message):
            return Interpretation("status_unavailable", reply="I have no live robot connection or status. I cannot verify its position, movement, speed, or ability to reverse.")
        if message in ("cancel", "cancel that", "forget that", "reset conversation"):
            self.reset()
            return Interpretation("conversation", reply="I cleared the discussed request. This does not cancel any real robot movement.")

        direct = self._request(message)
        if direct is not None:
            return direct
        followup = self._followup(message)
        if followup is not None:
            return followup
        if re.search(r"\b(?:and|then|while|also)\b", message):
            return clarification("Please describe one action at a time. A reverse request may include its distance or duration and speed together.")
        return clarification("I didn't understand a supported request. Try 'slow down', 'take the next right', or 'stop after two seconds'. Nothing was sent or executed.")

    def _obstacle_request(self, message):
        # Whole-message matching prevents descriptions, negation and compounds
        # from accidentally becoming an avoidance command.
        noun = r"(?:duck|ducks|obstacle|obstacles|yellow (?:object|duck|toy)|rubber duck|something|object)"
        target = r"(?:(?:a|an|the|that) )?" + noun
        location = r"(?:in (?:the |our |my )?(?:way|path|lane|road)|ahead|blocking (?:the |our |my )?(?:path|lane|road))"
        if re.fullmatch(r"(?:is there |do you see |can you see )" + target + r"(?: " + location + r")?", message):
            return Interpretation("status_unavailable", reply="I cannot see a live camera or confirm an obstacle. This is an offline text interpreter.")
        report = (re.fullmatch(target + r"(?: " + location + r")?", message)
                  or re.fullmatch(r"(?:there is |there are |there's |i see |i can see )" + target + r"(?: " + location + r")?", message)
                  or re.fullmatch(target + r" (?:is |are )" + location, message))
        if message in ("the path is blocked", "the road is blocked", "our lane is blocked",
                       "something is blocking the way", "there is something in front of us"):
            report = True
        if report:
            return complete("obstacle", {"intent": "report", "object": "obstacle",
                                         "source": "user_report", "camera_verified": False})
        if re.fullmatch(r"(?:avoid|go around|drive around|pass|overtake|get around|bypass) " + target, message):
            return complete("obstacle", {"intent": "avoid", "pass_side": "left",
                                         "return_lane": "right", "preserve_route": True})
        if re.fullmatch(r"(?:avoid|go around|pass|bypass) it", message):
            if self.last_request and self.last_request.category == "obstacle":
                return complete("obstacle", {"intent": "avoid", "pass_side": "left",
                                             "return_lane": "right", "preserve_route": True})
            return clarification("What should be avoided? Try 'go around the duck'.")
        if message in ("obstacle avoidance", "avoid obstacles automatically"):
            return complete("obstacle", {"intent": "avoid", "pass_side": "left",
                                         "return_lane": "right", "preserve_route": True})
        return None

    def _request(self, message):
        obstacle = self._obstacle_request(message)
        if obstacle is not None:
            return obstacle
        if message in ("stop", "halt", "wait", "stop now", "halt now", "stop immediately", "come to a stop"):
            return complete("stop", {"condition": "immediate"})
        if message in ("interrupt all movement", "interrupt all movement immediately", "interrupt movement", "stop everything", "stop all movement", "emergency stop", "cancel all movement", "halt everything"):
            return complete("interrupt", {"condition": "immediate", "cancel_pending_movement": True})
        match = re.fullmatch(r"(?:stop|halt) (after|in|for|at|until)(?: (.*))?", message)
        if match:
            relation, tail = match[1], match[2] or ""
            if relation == "until":
                return clarification("Do you mean stop upon reaching the next red line, or pause for a duration?", "stop", {"condition": "unresolved"}, ["condition"])
            if relation == "at" and re.fullmatch(CHECKPOINT, tail):
                return complete("stop", {"condition": "at_checkpoint", "checkpoint": "next_red_line"})
            if relation == "at":
                return clarification("Which checkpoint? I support the next red line.", "stop", {"condition": "at_checkpoint"}, ["checkpoint"])
            return self._limit("stop", {"condition": "pause" if relation == "for" else "after"}, tail)
        if re.fullmatch(r"stop (?:when|once) (?:we|you|the bot|the robot) (?:reach|reaches|get to) " + CHECKPOINT, message):
            return complete("stop", {"condition": "at_checkpoint", "checkpoint": "next_red_line"})

        match = re.fullmatch(r"(?:turn|go|take|take the next|next turn|turn at the next junction|at the next junction turn) (left|right|straight)(?: (?:at the next (?:turn|junction)|next))?", message)
        if match:
            return complete("turn", {"direction": match[1], "junction": "next"})
        if message in ("turn", "take the next turn", "turn at the next junction"):
            return clarification("Which direction at the next junction: left, right, or straight?", "turn", {"junction": "next"}, ["direction"])
        match = re.fullmatch(r"(speed up|go faster|faster|accelerate|pick up the pace|slow down|go slower|slower|decelerate|ease off|reduce speed)(?: (a little|a bit|slightly|by .+))?", message)
        if match:
            direction = "increase" if match[1] in ("speed up", "go faster", "faster", "accelerate", "pick up the pace") else "decrease"
            p = {"direction": direction}
            if match[2] and match[2].startswith("by "):
                amount = speed(match[2][3:])
                if not amount or amount.get("invalid"):
                    return clarification("Specify a positive speed change with units, for example 'by 10 percent'.")
                p["amount"] = amount
                if amount.get("unit") is None:
                    return clarification("What units for that speed change: percent, cm/s, or m/s?", "speed_change", p, ["amount_unit"])
            elif match[2]:
                p["degree"] = match[2]
            return complete("speed_change", p)
        match = re.fullmatch(r"(?:set (?:the )?speed to|go at) (.+)", message)
        if match:
            return self._with_speed("speed_setting", {}, match[1])
        match = re.fullmatch(r"(?:reverse|move backwards|move backward|go backwards|go backward|back up)(?: (.*))?", message)
        if match:
            tail = match[1] or ""
            p = {}
            if " at " in tail or tail.startswith("at "):
                tail, speed_text = (" " + tail).split(" at ", 1)
                tail = tail.strip()
                parsed = speed(speed_text)
                if not parsed or parsed.get("invalid"):
                    return clarification("Specify one reverse distance or duration and a positive speed with units.")
                p["speed"] = parsed
            else:
                qualifier = re.search(r" (very slowly|slowly|quickly)$", tail)
                if qualifier:
                    p["speed"] = {"description": qualifier[1]}
                    tail = tail[:qualifier.start()]
            tail = re.sub(r"^for ", "", tail)
            result = self._limit("reverse", p, tail)
            if not result.needs_clarification and p.get("speed", {}).get("unit", "known") is None:
                return clarification("What speed units do you mean: percent, cm/s, or m/s?", "reverse", result.parameters, ["speed_unit"])
            return result
        return None

    def _limit(self, category, parameters, text):
        p = copy.deepcopy(parameters)
        parsed = quantity(text)
        if not parsed or parsed.get("invalid"):
            question = ("Specify a positive pause duration, for example 'two seconds'."
                        if category == "stop" and p.get("condition") == "pause" else
                        "Specify a positive distance or duration, for example '20 centimetres' or 'two seconds'.")
            return clarification(question, category, p, ["distance_or_duration"])
        p.pop("distance_m", None)
        p.pop("duration_s", None)
        p.pop("unresolved_amount", None)
        p.update(parsed)
        if "unresolved_amount" in p:
            return clarification("What units do you mean, such as seconds or centimetres?", category, p, ["limit_unit"])
        if category == "stop":
            if p["condition"] == "pause" and "duration_s" not in p:
                return clarification("A pause needs a duration. Did you mean stop after travelling that distance instead?", category, p, ["duration"])
            if p["condition"] != "pause":
                p["condition"] = "after_duration" if "duration_s" in p else "after_distance"
        if p.get("speed", {}).get("unit", "known") is None:
            return clarification("What speed units do you mean: percent, cm/s, or m/s?", category, p, ["speed_unit"])
        return complete(category, p)

    def _with_speed(self, category, p, text):
        parsed = speed(text)
        if not parsed or parsed.get("invalid"):
            return clarification("Specify a positive speed with units, or say 'slowly'.", category, p, ["speed"])
        p = copy.deepcopy(p)
        p["speed"] = parsed
        if parsed.get("unit", "known") is None:
            return clarification("What speed units do you mean: percent, cm/s, or m/s?", category, p, ["speed_unit"])
        return complete(category, p)

    def _followup(self, message):
        previous = self.pending or self.last_request
        bare = re.sub(r"^(?:actually |make that |i mean |instead )", "", message)
        bare = re.sub(r" instead$", "", bare)
        if bare in ("left", "right", "straight", "the other direction", "the other way", "the opposite direction"):
            if not previous or previous.category != "turn":
                return clarification("Which turn are you referring to? Try 'take the next left'.")
            direction = bare
            if bare.startswith("the "):
                direction = {"left": "right", "right": "left"}.get(previous.parameters.get("direction"))
                if direction is None:
                    return clarification("Do you mean left or right at the next junction?")
            return complete("turn", {"direction": direction, "junction": "next"})
        if bare in ("a little more", "a bit more", "more", "same again", "again"):
            if not previous or previous.category != "speed_change" or previous.needs_clarification:
                return clarification("More of which speed adjustment? Say 'speed up' or 'slow down'.")
            p = copy.deepcopy(previous.parameters)
            if "more" in bare:
                p.pop("amount", None)
                p["degree"] = "a little"
            return complete("speed_change", p)
        if not previous:
            return None
        p = copy.deepcopy(previous.parameters)
        if previous.category == "stop" and p.get("condition") in ("unresolved", "at_checkpoint"):
            if re.fullmatch(r"(?:at |upon reaching )?" + CHECKPOINT, bare):
                return complete("stop", {"condition": "at_checkpoint", "checkpoint": "next_red_line"})
            match = re.fullmatch(r"(?:pause|wait|stop) for (.+)", bare)
            if match:
                return self._limit("stop", {"condition": "pause"}, match[1])
        if self.pending and "amount_unit" in previous.missing_fields and bare in SPEED_UNITS:
            p["amount"]["unit"] = SPEED_UNITS[bare]
            return complete(previous.category, p)
        if self.pending and p.get("speed", {}).get("unit", "known") is None and bare in SPEED_UNITS:
            p["speed"]["unit"] = SPEED_UNITS[bare]
            return complete(previous.category, p)
        if previous.category in ("reverse", "speed_setting"):
            if bare.startswith("at "):
                result = self._with_speed(previous.category, p, bare[3:])
                if previous.category == "reverse" and not ("distance_m" in p or "duration_s" in p):
                    result.needs_clarification = True
                    result.missing_fields = ["distance_or_duration"]
                    result.reply = "What reverse distance or duration do you mean?"
                return result
            if self.pending and "speed_unit" in previous.missing_fields and speed(bare):
                return self._with_speed(previous.category, p, bare)
        if previous.category in ("stop", "reverse"):
            if "unresolved_amount" in p and bare in set(DISTANCE) | set(TIME):
                bare = "%g %s" % (p["unresolved_amount"], bare)
            if quantity(bare):
                if previous.category == "stop" and p.get("condition") in ("immediate", "at_checkpoint", "unresolved"):
                    return clarification("Do you mean stop after that amount, or pause for that duration?")
                return self._limit(previous.category, p, bare)
        return None
