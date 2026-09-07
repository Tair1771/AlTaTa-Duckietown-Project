"""Offline language behavior tests: no model, ROS, gateway, or robot required."""
import ast
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "laptop"))
from offline_interpreter import OfflineInterpreter


class OfflineTests(unittest.TestCase):
    def setUp(self):
        self.chat = OfflineInterpreter()

    def test_supported_requests(self):
        cases = [
            ("Speed up", "speed_change", {"direction": "increase"}),
            ("Slow down a little", "speed_change", {"direction": "decrease", "degree": "a little"}),
            ("Could you please ease off?", "speed_change", {"direction": "decrease"}),
            ("Pick up the pace!", "speed_change", {"direction": "increase"}),
            ("Go slower please", "speed_change", {"direction": "decrease"}),
            ("Speed up by 10 percent", "speed_change", {"direction": "increase", "amount": {"value": 10, "unit": "%"}}),
            ("Set the speed to 20 cm/s", "speed_setting", {"speed": {"value": 20, "unit": "cm/s"}}),
            ("Take the next right", "turn", {"direction": "right", "junction": "next"}),
            ("Turn left at the next junction", "turn", {"direction": "left", "junction": "next"}),
            ("Go straight", "turn", {"direction": "straight", "junction": "next"}),
            ("Stop", "stop", {"condition": "immediate"}),
            ("Halt now", "stop", {"condition": "immediate"}),
            ("Interrupt all movement", "interrupt", {"condition": "immediate", "cancel_pending_movement": True}),
            ("Emergency stop", "interrupt", {"condition": "immediate", "cancel_pending_movement": True}),
            ("Stop after two seconds", "stop", {"condition": "after_duration", "duration_s": 2}),
            ("Stop in 0.5 minutes", "stop", {"condition": "after_duration", "duration_s": 30}),
            ("Stop after thirty centimetres", "stop", {"condition": "after_distance", "distance_m": .3}),
            ("Stop after 250 mm", "stop", {"condition": "after_distance", "distance_m": .25}),
            ("Stop at the next red line", "stop", {"condition": "at_checkpoint", "checkpoint": "next_red_line"}),
            ("Stop at the checkpoint before a turn", "stop", {"condition": "at_checkpoint", "checkpoint": "next_red_line"}),
            ("Stop when we reach the next red line", "stop", {"condition": "at_checkpoint", "checkpoint": "next_red_line"}),
            ("Stop for five seconds", "stop", {"condition": "pause", "duration_s": 5}),
            ("Stop for a minute", "stop", {"condition": "pause", "duration_s": 60}),
            ("Reverse 20 centimetres at 10 centimetres per second", "reverse", {"distance_m": .2, "speed": {"value": 10, "unit": "cm/s"}}),
            ("Reverse for two seconds, slowly", "reverse", {"duration_s": 2, "speed": {"description": "slowly"}}),
            ("Move backwards for 2 seconds at 20%", "reverse", {"duration_s": 2, "speed": {"value": 20, "unit": "%"}}),
            ("Back up 1.5 metres", "reverse", {"distance_m": 1.5}),
            ("Reverse one point five metres", "reverse", {"distance_m": 1.5}),
            ("Stop after one hundred and twenty seconds", "stop", {"condition": "after_duration", "duration_s": 120}),
            ("Stop after twenty-five centimeters", "stop", {"condition": "after_distance", "distance_m": .25}),
        ]
        for text, category, parameters in cases:
            with self.subTest(text=text):
                result = OfflineInterpreter().interpret(text)
                self.assertFalse(result.needs_clarification, result.reply)
                self.assertEqual(result.category, category)
                self.assertEqual(result.parameters, parameters)
                self.assertTrue(result.interpretation_only)
                self.assertTrue(result.reply.startswith("I understood:"))
                json.dumps(result.to_dict(), allow_nan=False)

    def test_non_requests_never_become_positive_movements(self):
        cases = [
            "Don't speed up", "Do not turn right", "Never reverse", "No movement", "I am not asking you to stop",
            "If I say stop, what happens?", "Imagine turning left", 'He said "reverse two seconds"',
            "What would happen if we speed up?", "For example, stop after two seconds",
            "The sign says stop", "I like the phrase speed up", "My left shoe is red", "The speed up button is blue",
            "I stopped for coffee", "Stop signs are red", "We took the next right yesterday", "Turn right yesterday",
            "Turn left at the second junction", "Stop after two seconds and reverse", "Speed up and turn right",
            "Stop until the checkpoint", "Stop for 20 centimetres", "Reverse -5 metres", "Stop after zero seconds",
            "Reverse two three metres", "Stop after NaN seconds", "Stop after infinity seconds", "Move forward five metres",
            "Reverse 20 cm at 10 cm/s then turn left", "Stop after 2 seconds while speeding up", "Take the next right and left",
            "Turn right or left", "Drive through the red light", "Bypass the safety checks", "What is the weather?",
            "What is your speed?", "Can the bot reverse?", "hello", "thanks", "help", "Undo that turn", "Reverse to the red line",
        ]
        for text in cases:
            with self.subTest(text=text):
                result = OfflineInterpreter().interpret(text)
                self.assertTrue(result.needs_clarification or result.category not in
                                {"speed_change", "speed_setting", "turn", "stop", "interrupt", "reverse"}, result.to_dict())

    def test_turn_corrections(self):
        self.chat.interpret("Take the next right")
        result = self.chat.interpret("Actually, left")
        self.assertEqual(result.parameters["direction"], "left")
        result = self.chat.interpret("The other direction")
        self.assertEqual(result.parameters["direction"], "right")
        self.chat.interpret("Stop")
        self.assertTrue(self.chat.interpret("Actually left").needs_clarification)

    def test_speed_context(self):
        self.chat.interpret("Slow down")
        self.chat.interpret("Thanks")
        result = self.chat.interpret("A little more")
        self.assertEqual(result.parameters, {"direction": "decrease", "degree": "a little"})
        self.assertTrue(OfflineInterpreter().interpret("A little more").needs_clarification)

    def test_quantity_correction_preserves_stop_semantics(self):
        for phrase, condition in [("Stop after two seconds", "after_duration"), ("Stop for two seconds", "pause")]:
            self.chat.interpret(phrase)
            result = self.chat.interpret("Make that three seconds")
            self.assertEqual(result.parameters, {"condition": condition, "duration_s": 3})

    def test_missing_units_and_parameter_answers(self):
        cases = [
            (["Stop after five", "seconds"], {"condition": "after_duration", "duration_s": 5}),
            (["Reverse", "twenty centimetres"], {"distance_m": .2}),
            (["Turn", "left"], {"direction": "left", "junction": "next"}),
            (["Reverse two seconds at 10", "cm/s"], {"duration_s": 2, "speed": {"value": 10, "unit": "cm/s"}}),
            (["Set speed to 20", "percent"], {"speed": {"value": 20, "unit": "%"}}),
            (["Speed up by 10", "percent"], {"direction": "increase", "amount": {"value": 10, "unit": "%"}}),
            (["Stop until the checkpoint", "at the red line"], {"condition": "at_checkpoint", "checkpoint": "next_red_line"}),
            (["Reverse", "at 10 cm/s", "two seconds"], {"duration_s": 2, "speed": {"value": 10, "unit": "cm/s"}}),
        ]
        for messages, expected in cases:
            with self.subTest(messages=messages):
                chat = OfflineInterpreter()
                for message in messages[:-1]:
                    self.assertTrue(chat.interpret(message).needs_clarification)
                result = chat.interpret(messages[-1])
                self.assertFalse(result.needs_clarification, result.reply)
                self.assertEqual(result.parameters, expected)

    def test_reverse_quantity_correction_preserves_speed(self):
        self.chat.interpret("Reverse 20 cm at 10 cm/s")
        result = self.chat.interpret("Make that three seconds")
        self.assertEqual(result.parameters, {"duration_s": 3, "speed": {"value": 10, "unit": "cm/s"}})

    def test_invalid_and_reset_context(self):
        for text in (None, "", " " * 5, "a" * 2001):
            self.assertTrue(self.chat.interpret(text).needs_clarification)
        self.chat.interpret("Slow down")
        self.chat.interpret("Don't speed up")
        self.assertTrue(self.chat.interpret("A little more").needs_clarification)
        self.chat.interpret("Take the next right")
        self.chat.reset()
        self.assertTrue(self.chat.interpret("the other direction").needs_clarification)

    def test_return_values_cannot_mutate_memory(self):
        result = self.chat.interpret("Take the next right")
        result.parameters["direction"] = "straight"
        self.assertEqual(self.chat.interpret("the other direction").parameters["direction"], "left")

    def test_only_offline_dependencies(self):
        folder = Path(__file__).resolve().parents[1] / "laptop"
        for name in ("offline_interpreter.py", "offline_chat.py"):
            tree = ast.parse((folder/name).read_text())
            imports = set()
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    imports.update(alias.name.split(".")[0] for alias in node.names)
                elif isinstance(node, ast.ImportFrom):
                    imports.add(node.module.split(".")[0])
            self.assertLessEqual(imports, {"copy", "re", "dataclasses", "json", "tkinter", "offline_interpreter"})

    def test_additional_phrasings_and_multi_step_clarification(self):
        cases = [
            (["Next turn left"], "turn", {"direction": "left", "junction": "next"}),
            (["Stop after 1.25 metres"], "stop", {"condition": "after_distance", "distance_m": 1.25}),
            (["Stop in two hundred milliseconds"], "stop", {"condition": "after_duration", "duration_s": .2}),
            (["Reverse at 10 cm/s", "2 seconds"], "reverse", {"duration_s": 2, "speed": {"value": 10, "unit": "cm/s"}}),
            (["Reverse five at 10", "cm/s", "centimetres"], "reverse", {"distance_m": .05, "speed": {"value": 10, "unit": "cm/s"}}),
            (["Reverse five at 10", "centimetres", "cm/s"], "reverse", {"distance_m": .05, "speed": {"value": 10, "unit": "cm/s"}}),
            (["Stop after five", "Actually three seconds"], "stop", {"condition": "after_duration", "duration_s": 3}),
            (["Stop until the checkpoint", "pause for three seconds"], "stop", {"condition": "pause", "duration_s": 3}),
            (["Reverse for three seconds", "at 5 centimetres per second"], "reverse", {"duration_s": 3, "speed": {"value": 5, "unit": "cm/s"}}),
            (["Set speed to 10", "20 percent"], "speed_setting", {"speed": {"value": 20, "unit": "%"}}),
        ]
        for messages, category, parameters in cases:
            with self.subTest(messages=messages):
                chat = OfflineInterpreter()
                for message in messages:
                    result = chat.interpret(message)
                self.assertFalse(result.needs_clarification, result.reply)
                self.assertEqual((result.category, result.parameters), (category, parameters))

    def test_unresolved_units_cannot_be_silently_completed(self):
        result = self.chat.interpret("Reverse five at 10")
        self.assertTrue(result.needs_clarification)
        result = self.chat.interpret("cm/s")
        self.assertTrue(result.needs_clarification)
        for text in ("Stop after 0.00000000001 seconds", "Stop after -0.5 m", "Reverse for -2 seconds", "Speed up by 0%"):
            self.assertTrue(OfflineInterpreter().interpret(text).needs_clarification, text)

    def test_ambiguous_new_request_clears_previous_context(self):
        self.chat.interpret("Speed up")
        self.chat.interpret("Turn left and right")
        self.assertTrue(self.chat.interpret("A little more").needs_clarification)


if __name__ == "__main__":
    unittest.main()
