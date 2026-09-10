"""Session policy only; wheel steering and junction profiles live in the node."""
import math

PROFILES = {"slow": 0.09, "normal": 0.10, "fast": 0.11}
STRAIGHT_WHEEL_CAP = 0.12
WAIT_SECONDS = 30.0


class LiveSession:
    def __init__(self):
        self.enabled = False
        self.active = False
        self.run_id = None
        self.profile = "normal"
        self.pause_pending = False
        self.pause_seconds = None
        self.paused_at = None
        self.resume_at = None
        self.straight_since = None
        self.straight = False
        self.red_wait_at = None
        self.red_index = None
        self.instruction_id = None
        self.end_reason = None

    def start(self, run_id):
        if not isinstance(run_id, str) or not 1 <= len(run_id) <= 100:
            raise ValueError("A run identifier is required")
        self.__init__()
        self.enabled = self.active = True
        self.run_id = run_id

    def end(self, reason):
        self.active = False
        self.pause_pending = False
        self.paused_at = self.resume_at = None
        self.straight = False
        self.straight_since = None
        self.end_reason = reason

    def request_pause(self, seconds=None):
        if seconds is not None and (isinstance(seconds, bool)
                or not isinstance(seconds, (int, float))
                or not math.isfinite(seconds) or not 0 < seconds <= 3600):
            raise ValueError("Pause duration must be greater than zero and at most 3600 seconds")
        # A second pause must not silently extend an existing pause/deadline.
        if self.paused_at is not None:
            raise ValueError("Already paused; say continue before requesting another pause")
        self.pause_pending = True
        self.pause_seconds = seconds

    def resume(self):
        self.pause_pending = False
        self.pause_seconds = None
        self.paused_at = self.resume_at = None

    def observe_straight(self, now, eligible):
        if not eligible:
            self.straight_since = None
            self.straight = False
        else:
            if self.straight_since is None:
                self.straight_since = now
            self.straight = now - self.straight_since >= 0.5

    def tick(self, now, state, index, stop_started, healthy):
        if not self.active:
            return
        if state == "red_stop":
            if self.red_index != index:
                self.red_index = index
                self.red_wait_at = now if stop_started is None else stop_started
            if now - self.red_wait_at >= WAIT_SECONDS:
                self.end("No junction instruction within 30 seconds of the red stop")
                return
        else:
            self.red_wait_at = None
            self.red_index = None
        if self.paused_at is not None:
            if self.resume_at is None:
                if now - self.paused_at >= WAIT_SECONDS:
                    self.end("No Continue within 30 seconds of pausing")
            elif now >= self.resume_at:
                if healthy:
                    self.resume()
                else:
                    self.end("Timed pause ended without a healthy connection and fresh camera")
        elif self.pause_pending and self.straight and healthy and state == "following":
            self.pause_pending = False
            self.paused_at = now
            self.resume_at = None if self.pause_seconds is None else now + self.pause_seconds

    def snapshot(self, now):
        return {
            "version": 1, "enabled": self.enabled, "active": self.active,
            "run_id": self.run_id, "profile": self.profile,
            "straight": self.straight, "pause_pending": self.pause_pending,
            "paused": self.paused_at is not None,
            "pause_remaining": None if self.resume_at is None else max(0, self.resume_at-now),
            "wait_remaining": None if self.red_wait_at is None else max(0, WAIT_SECONDS-(now-self.red_wait_at)),
            "instruction_id": self.instruction_id, "end_reason": self.end_reason,
        }
