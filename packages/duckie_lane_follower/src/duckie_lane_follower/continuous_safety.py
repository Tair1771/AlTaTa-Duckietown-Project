"""Pure state checks used by the independent continuous-route watchdog."""

COMMAND_THRESHOLD = 0.07
FEEDBACK_TIMEOUT = 0.5
STALL_TIMEOUT = 0.75


class SafetyState:
    def __init__(self, started_at):
        self.started_at = started_at
        self.status = None
        self.status_at = None
        self.request = (0.0, 0.0)
        self.request_at = None
        self.executed = (0.0, 0.0)
        self.executed_at = None
        self.encoder = {"left": (None, None), "right": (None, None)}
        self.motion = {"left": None, "right": None}

    def record_status(self, now, value):
        self.status, self.status_at = value, now

    def record_request(self, now, left, right):
        self.request, self.request_at = (float(left), float(right)), now

    def record_executed(self, now, left, right):
        self.executed, self.executed_at = (float(left), float(right)), now
        for side, value in zip(("left", "right"), self.executed):
            if abs(value) >= COMMAND_THRESHOLD:
                if self.motion[side] is None:
                    self.motion[side] = (now, self.encoder[side][0])
            else:
                self.motion[side] = None

    def record_encoder(self, now, side, ticks):
        ticks = int(ticks)
        previous = self.encoder[side][0]
        self.encoder[side] = (ticks, now)
        motion = self.motion[side]
        # Measure the time since the most recent encoder progress. Without this,
        # one early tick would hide a later sustained stall for the whole route.
        if motion is not None and previous is not None and ticks != previous:
            self.motion[side] = (now, ticks)

    def fault(self, now, wheel_publishers):
        if now - self.started_at < 2.0:
            return None
        if wheel_publishers != ["/lane_follower_node"]:
            return "Wheel publisher ownership changed"
        active = any(abs(value) >= COMMAND_THRESHOLD
                     for value in self.request + self.executed)
        if not active:
            return None
        for label, updated in (("wheel request", self.request_at),
                               ("executed-wheel feedback", self.executed_at),
                               ("lane status", self.status_at)):
            if updated is None or now - updated > FEEDBACK_TIMEOUT:
                return "%s became stale" % label.capitalize()
        if not isinstance(self.status, dict) or self.status.get("camera_valid") is not True:
            return "Lane follower rejected the camera"
        for side in ("left", "right"):
            motion = self.motion[side]
            if motion is None or now - motion[0] < STALL_TIMEOUT:
                continue
            ticks, updated = self.encoder[side]
            if updated is None or now - updated > FEEDBACK_TIMEOUT:
                return "%s wheel encoder feedback became stale" % side.capitalize()
            if motion[1] is None or ticks == motion[1]:
                return "%s wheel stalled while commanded" % side.capitalize()
        return None
