"""Exhaustive offline checks for directed right-lane A* planning."""

from collections import deque
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "laptop"))

from route_planner import (MAP_ID, MAP_PORTS, RED_LINE_APPROACHES,
                           junction_turn, parse_approach, plan_route)


def independent_shortest(start, goal, first_turn=None):
    start, goal = parse_approach(start), parse_approach(goal)
    queue = deque([(start, 0)])
    seen = {start}
    while queue:
        (previous, junction), distance = queue.popleft()
        if (previous, junction) == goal:
            return distance
        for following in MAP_PORTS[junction].values():
            if following == previous:
                continue
            if distance == 0 and first_turn and junction_turn(
                    previous, junction, following) != first_turn:
                continue
            state = (junction, following)
            if state not in seen:
                seen.add(state)
                queue.append((state, distance + 1))
    return None


class RoutePlannerTests(unittest.TestCase):
    def test_every_red_line_pair_matches_exhaustive_shortest_path(self):
        self.assertEqual(len(RED_LINE_APPROACHES), 16)
        for start in RED_LINE_APPROACHES:
            for destination in RED_LINE_APPROACHES:
                plan = plan_route(start, destination)
                self.assertEqual(plan.junction_count,
                                 independent_shortest(start, destination))
                self.assertEqual(plan.start_approach, start)
                self.assertEqual(plan.destination_approach, destination)
                self.assertEqual("%s->%s" % plan.route[-2:], destination)
                self.assertNotIn("uturn", plan.turns)
                self.assertEqual(plan.map_id, MAP_ID)

    def test_exact_approach_matters_and_curves_do_not_add_cost(self):
        self.assertEqual(plan_route("A->B", "A->B").junction_count, 0)
        self.assertEqual(plan_route("A->B", "B->A").junction_count, 4)
        self.assertEqual(plan_route("A->B", "B->C").turns, ("straight",))

    def test_first_turn_constraint_or_impossible_exit(self):
        plan = plan_route("A->B", "D->A", required_first_turn="left")
        self.assertEqual(plan.turns[0], "left")
        with self.assertRaisesRegex(ValueError, "No legal right-lane route"):
            plan_route("B->D", "B->C", required_first_turn="straight")

    def test_invalid_approaches_and_turns(self):
        for value in ("A", "A->Q", "A->C", None):
            with self.assertRaises(ValueError):
                plan_route(value, "A->B")
        with self.assertRaises(ValueError):
            plan_route("A->B", "B->C", required_first_turn="reverse")

    def test_equal_cost_result_is_deterministic(self):
        routes = {plan_route("A->B", "D->C").route for _ in range(20)}
        self.assertEqual(len(routes), 1)


if __name__ == "__main__":
    unittest.main()
