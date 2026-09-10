"""Session policy only; wheel steering and junction profiles live in the node."""
import math
import time
from .route_map import approach_id, parse_approach

PROFILES = {"slow": 0.09, "normal": 0.10, "fast": 0.11}
STRAIGHT_WHEEL_CAP = 0.12
WAIT_SECONDS = 30.0
RED_WAIT_SECONDS = 60.0


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
        self.pause_accounted_at = None
        self.straight_since = None
        self.straight = False
        self.red_wait_at = None
        self.red_index = None
        self.instruction_id = None
        self.end_reason = None
        self.stop_at_next_red = False
        self.finish_approach = None
        self.stop_after_junction = False
        self.finish_after_junction_red = False
        self.center_initial_straight = False

    def start(self, run_id, stop_at_next_red=False, finish_approach=None, stop_after_junction=False,
              finish_after_junction_red=False, center_initial_straight=False):
        if not isinstance(run_id, str) or not 1 <= len(run_id) <= 100:
            raise ValueError("A run identifier is required")
        if not isinstance(stop_at_next_red, bool):
            raise ValueError("stop_at_next_red must be boolean")
        if not isinstance(stop_after_junction, bool):
            raise ValueError("stop_after_junction must be boolean")
        if stop_after_junction and (stop_at_next_red or finish_approach is not None):
            raise ValueError("Choose only one run completion condition")
        if not isinstance(finish_after_junction_red, bool) or not isinstance(center_initial_straight, bool):
            raise ValueError("Initial-straight and final-red options must be boolean")
        if finish_after_junction_red and (stop_after_junction or stop_at_next_red or finish_approach is not None):
            raise ValueError("Choose only one run completion condition")
        finish = (approach_id(*parse_approach(finish_approach))
                  if finish_approach is not None else None)
        if stop_at_next_red and finish is not None:
            raise ValueError("Choose either the next red line or a final directed red line")
        self.__init__()
        self.enabled = self.active = True
        self.run_id = run_id
        self.stop_at_next_red = stop_at_next_red
        self.finish_approach = finish
        self.stop_after_junction = stop_after_junction
        self.finish_after_junction_red = finish_after_junction_red
        self.center_initial_straight = center_initial_straight

    def end(self, reason):
        self.active = False
        self.pause_pending = False
        self.paused_at = self.resume_at = None
        self.pause_accounted_at = None
        self.straight = False
        self.straight_since = None
        self.end_reason = reason

    def request_pause(self, seconds=None, now=None):
        if seconds is not None and (isinstance(seconds, bool)
                or not isinstance(seconds, (int, float))
                or not math.isfinite(seconds) or not 0 < seconds <= 3600):
            raise ValueError("Pause duration must be greater than zero and at most 3600 seconds")
        # A second pause must not silently extend an existing pause/deadline.
        if self.paused_at is not None:
            raise ValueError("Already paused; say continue before requesting another pause")
        # Stop at receipt, including curves and unmarked junctions. The node
        # publishes zero under its wheel lock before acknowledging this request.
        now = time.monotonic() if now is None else now
        self.pause_pending = False
        self.pause_seconds = seconds
        self.paused_at = self.pause_accounted_at = now
        self.resume_at = None if seconds is None else now + seconds

    def account_pause(self, now):
        """Return newly paused time; never use it to age camera/heartbeat data."""
        if self.paused_at is None:
            return 0.0
        elapsed = max(0.0, now - self.pause_accounted_at)
        self.pause_accounted_at = now
        if self.red_wait_at is not None:
            self.red_wait_at += elapsed
        return elapsed

    def resume(self):
        self.pause_pending = False
        self.pause_seconds = None
        self.paused_at = self.resume_at = None
        self.pause_accounted_at = None

    def observe_straight(self, now, eligible):
        if not eligible:
            self.straight_since = None
            self.straight = False
        else:
            if self.straight_since is None:
                self.straight_since = now
            self.straight = now - self.straight_since >= 0.5

    def tick(self, now, state, index, stop_started, healthy, approach=None):
        if not self.active:
            return
        self.account_pause(now)
        if self.stop_after_junction and index >= 2 and state in ("following", "red_stop"):
            self.end("Single straight crossing complete; outgoing lane reacquired")
            return
        if state == "red_stop":
            if self.finish_after_junction_red and index >= 2:
                self.end("Destination reached at %s red line" % approach)
                return
            if self.finish_approach is not None and approach == self.finish_approach:
                self.end("Destination reached at %s red line" % approach)
                return
            if self.stop_at_next_red:
                self.end("Pause check complete at red line")
                return
            if self.red_index != index:
                self.red_index = index
                self.red_wait_at = now if stop_started is None else stop_started
            if now - self.red_wait_at >= RED_WAIT_SECONDS:
                self.end("No junction instruction within 60 seconds of the red stop")
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

    def snapshot(self, now):
        return {
            "version": 1, "enabled": self.enabled, "active": self.active,
            "pause_mode": "immediate",
            "stop_at_next_red": self.stop_at_next_red,
            "supports_pause_check": True,
            "supports_finish_approach": True,
            "supports_stop_after_junction": True,
            "stop_after_junction": self.stop_after_junction,
            "supports_initial_straight_check": True,
            "finish_after_junction_red": self.finish_after_junction_red,
            "center_initial_straight": self.center_initial_straight,
            "finish_approach": self.finish_approach,
            "red_wait_seconds": RED_WAIT_SECONDS,
            "run_id": self.run_id, "profile": self.profile,
            "straight": self.straight, "pause_pending": self.pause_pending,
            "paused": self.paused_at is not None,
            "pause_remaining": None if self.resume_at is None else max(0, self.resume_at-now),
            "wait_remaining": None if self.red_wait_at is None else max(0, RED_WAIT_SECONDS-(now-self.red_wait_at)),
            "instruction_id": self.instruction_id, "end_reason": self.end_reason,
        }
