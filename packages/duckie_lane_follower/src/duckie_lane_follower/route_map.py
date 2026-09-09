#!/usr/bin/env python3
"""Pure map model and directed A* planner for the supplied AlTaTa track map."""

from dataclasses import dataclass
import heapq


MAP_ID = "altata-five-junction-v1"
HEADINGS = ("N", "E", "S", "W")

# Port directions describe the road at each junction, including curved roads.
MAP_PORTS = {
    "A": {"N": "D", "E": "B", "S": "E"},
    "B": {"N": "D", "E": "C", "S": "E", "W": "A"},
    "C": {"N": "D", "S": "E", "W": "B"},
    "D": {"N": "A", "S": "C", "W": "B"},
    "E": {"N": "B", "E": "C", "W": "A"},
}

# Coordinates and centre paths are schematic; they are not physical distances.
MAP_NODES = {"A": (65, 300), "B": (250, 300), "C": (410, 300),
             "D": (410, 175), "E": (250, 455)}
MAP_ROADS = {
    ("A", "B"): (65, 300, 250, 300),
    ("B", "C"): (250, 300, 410, 300),
    ("B", "D"): (250, 300, 250, 215, 285, 175, 410, 175),
    ("C", "D"): (410, 300, 410, 175),
    ("B", "E"): (250, 300, 250, 455),
    ("A", "E"): (65, 300, 65, 420, 95, 455, 250, 455),
    ("C", "E"): (410, 300, 410, 420, 380, 455, 250, 455),
    ("A", "D"): (65, 300, 65, 220, 95, 195, 130, 195, 150, 170,
                 150, 130, 180, 105, 180, 70, 215, 45, 370, 45,
                 410, 80, 410, 175),
}


def approach_id(previous, junction):
    """Identify travel from previous toward the red line before junction."""
    if previous not in MAP_PORTS or junction not in MAP_PORTS[previous].values():
        raise ValueError("Unknown directed lane segment")
    return "%s->%s" % (previous, junction)


def parse_approach(value):
    if not isinstance(value, str):
        raise ValueError("Approach must look like A->B")
    parts = [part.strip().upper() for part in value.replace("→", "->").split("->")]
    if len(parts) != 2:
        raise ValueError("Approach must look like A->B")
    approach_id(*parts)
    return tuple(parts)


RED_LINE_APPROACHES = tuple(sorted(
    approach_id(previous, junction)
    for previous, ports in MAP_PORTS.items()
    for junction in ports.values()
))


def validate_route(route):
    if not isinstance(route, list) or not 2 <= len(route) <= 50:
        raise ValueError("Route must contain 2 to 50 junction names")
    if any(not isinstance(j, str) or j not in MAP_PORTS for j in route):
        raise ValueError("Unknown map junction")
    for first, second in zip(route, route[1:]):
        if second not in MAP_PORTS[first].values():
            raise ValueError("No road from %s to %s" % (first, second))
    if any(first == third for first, third in zip(route, route[2:])):
        raise ValueError("U-turns are not supported")
    return list(route)


def junction_turn(previous, junction, following):
    entry = next(port for port, neighbor in MAP_PORTS[junction].items()
                 if neighbor == previous)
    exit_port = next(port for port, neighbor in MAP_PORTS[junction].items()
                     if neighbor == following)
    incoming = (HEADINGS.index(entry) + 2) % 4
    delta = (HEADINGS.index(exit_port) - incoming) % 4
    return {0: "straight", 1: "right", 3: "left"}.get(delta, "uturn")


def route_via_turn(route, index, turn):
    """Change one route exit and reconnect to the existing next waypoint."""
    if index >= len(route) - 1:
        raise ValueError("The next junction is the route destination")
    previous, junction, target = route[index-1:index+2]
    choices = sorted(neighbor for neighbor in MAP_PORTS[junction].values()
                     if neighbor != previous
                     and junction_turn(previous, junction, neighbor) == turn)
    if not choices:
        raise ValueError("No %s exit at junction %s" % (turn, junction))
    queue = [[junction, choices[0]]]
    while queue:
        path = queue.pop(0)
        if path[-1] == target:
            candidate = route[:index] + path + route[index+2:]
            try:
                return validate_route(candidate)
            except ValueError:
                continue
        for neighbor in sorted(MAP_PORTS[path[-1]].values()):
            if neighbor not in path and neighbor != path[-2]:
                queue.append(path + [neighbor])
    raise ValueError("Cannot reconnect this turn to the remaining route")


def _relaxed_distance(start, goal):
    """Admissible junction-count heuristic that ignores incoming direction."""
    if start == goal:
        return 0
    queue = [(start, 0)]
    seen = {start}
    while queue:
        node, distance = queue.pop(0)
        for neighbor in sorted(MAP_PORTS[node].values()):
            if neighbor == goal:
                return distance + 1
            if neighbor not in seen:
                seen.add(neighbor)
                queue.append((neighbor, distance + 1))
    return 0


@dataclass(frozen=True)
class RoutePlan:
    map_id: str
    start_approach: str
    destination_approach: str
    route: tuple
    turns: tuple
    junction_count: int

    def to_dict(self):
        return {
            "map_id": self.map_id,
            "start_approach": self.start_approach,
            "destination_approach": self.destination_approach,
            "route": list(self.route),
            "turns": list(self.turns),
            "junction_count": self.junction_count,
            "cost_basis": "junction_count",
            "offline_only": True,
        }


def plan_route(start_approach, destination_approach, required_first_turn=None):
    """Find a legal right-lane route between exact incoming approaches."""
    start = parse_approach(start_approach)
    goal = parse_approach(destination_approach)
    if required_first_turn not in (None, "left", "right", "straight"):
        raise ValueError("First turn must be left, right, or straight")

    # State is (previous junction, junction being approached). Each transition
    # crosses the current junction, so every transition costs one.
    frontier = [(0, 0, (approach_id(*start),), start, (start[0], start[1]))]
    best = {start: 0}
    while frontier:
        _, cost, _, state, route = heapq.heappop(frontier)
        if cost != best.get(state):
            continue
        if state == goal:
            turns = tuple(junction_turn(*triple)
                          for triple in zip(route, route[1:], route[2:]))
            return RoutePlan(MAP_ID, approach_id(*start), approach_id(*goal),
                             tuple(route), turns, cost)
        previous, junction = state
        for following in sorted(MAP_PORTS[junction].values()):
            if following == previous:
                continue
            turn = junction_turn(previous, junction, following)
            if cost == 0 and required_first_turn is not None and turn != required_first_turn:
                continue
            next_state = (junction, following)
            next_cost = cost + 1
            if next_cost >= best.get(next_state, float("inf")):
                continue
            best[next_state] = next_cost
            next_route = route + (following,)
            key = tuple(approach_id(a, b) for a, b in zip(next_route, next_route[1:]))
            estimate = next_cost + _relaxed_distance(following, goal[1])
            heapq.heappush(frontier, (estimate, next_cost, key, next_state, next_route))
    detail = (" with first turn %s" % required_first_turn) if required_first_turn else ""
    raise ValueError("No legal right-lane route reaches %s%s" %
                     (approach_id(*goal), detail))
