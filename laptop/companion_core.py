"""Local map-selection state for the companion; no robot transport."""

from dataclasses import asdict, dataclass, field

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
    """Own a route draft before the operator confirms Start."""

    def __init__(self):
        self.start_approach = None
        self.destination_approach = None
        self.plan = None
        self.draft_valid = False
        self.history = []

    def reset(self):
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
