"""Load the project's shared, pure route model for the Windows companion."""

import sys
from pathlib import Path

_SHARED = Path(__file__).resolve().parents[1] / "packages" / "duckie_lane_follower" / "src"
if str(_SHARED) not in sys.path:
    sys.path.insert(0, str(_SHARED))

from duckie_lane_follower.route_map import (
    HEADINGS, MAP_ID, MAP_NODES, MAP_PORTS, MAP_ROADS,
    RED_LINE_APPROACHES, RoutePlan, approach_id, junction_turn,
    parse_approach, plan_route, route_via_turn, validate_route)

__all__ = [
    "HEADINGS", "MAP_ID", "MAP_NODES", "MAP_PORTS", "MAP_ROADS",
    "RED_LINE_APPROACHES", "RoutePlan", "approach_id", "junction_turn",
    "parse_approach", "plan_route", "route_via_turn", "validate_route",
]
