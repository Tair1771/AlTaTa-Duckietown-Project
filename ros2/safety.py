"""Fail-closed, transport-independent wheel-request lease gate.

The receiver owns the monotonic clock, session, epoch and absolute expiry.
Renewal requires a new arm operation, never merely receiving another request.
"""
import math
import secrets


class SafetyGate:
    def __init__(self, clock, max_speed=.05, timeout=.25, arm_duration=8.):
        if not (0 < max_speed <= .05 and 0 < timeout <= .5 and 0 < arm_duration <= 8):
            raise ValueError("Unsafe supervisor limits")
        self.clock = clock
        self.max_speed, self.timeout, self.arm_duration = max_speed, timeout, arm_duration
        self.epoch = 0
        self.session = secrets.token_hex(16)
        self.armed = False
        self.reason = "not_armed"
        self.last_sequence = -1
        self.deadline = self.request_deadline = 0.
        self.wheels = (0., 0.)

    def stop(self, reason="stop"):
        self.epoch += 1
        self.armed = False
        self.wheels = (0., 0.)
        self.reason = reason
        return self.wheels

    def arm(self, ready):
        self.stop("not_ready")
        if ready is not True:
            return False
        self.armed = True
        self.reason = "armed"
        self.last_sequence = -1
        self.deadline = self.clock() + self.arm_duration
        self.request_deadline = self.clock() + self.timeout
        return True

    def accept(self, request):
        now = self.clock()
        self.output()
        if not self.armed:
            return False
        try:
            if not isinstance(request, dict):
                raise ValueError("request must be an object")
            if request.get("session") != self.session or request.get("epoch") != self.epoch:
                return False  # An old epoch cannot affect a newly armed session.
            seq = request["sequence"]
            if type(seq) is not int or seq <= self.last_sequence:
                return False
            issued = request["issued_monotonic"]
            values = (issued, request["left"], request["right"])
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
                raise ValueError("nonfinite request")
            if not 0 <= now - issued < self.timeout:
                raise ValueError("expired request")
            if any(not 0 <= v <= self.max_speed for v in values[1:]):
                raise ValueError("wheel limit exceeded")
            self.last_sequence = seq
            self.request_deadline = issued + self.timeout
            self.wheels = tuple(values[1:])
            return True
        except (KeyError, ValueError, TypeError):
            self.stop("invalid_request")
            return False

    def output(self):
        now = self.clock()
        if self.armed and (now >= self.deadline or now >= self.request_deadline):
            self.stop("lease_expired")
        return self.wheels if self.armed else (0., 0.)

    def status(self):
        self.output()
        return dict(session=self.session, epoch=self.epoch, armed=self.armed,
                    reason=self.reason, diagnostic_only=True)
