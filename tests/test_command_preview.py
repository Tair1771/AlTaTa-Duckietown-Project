"""Offline command translation and native window lifecycle tests."""
import ast
import builtins
import socket
import subprocess
import sys
import unittest
import urllib.request
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "laptop"))
from command_preview import PreviewSession, Preview, FakeReceiver


class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.session = PreviewSession()

    def test_mappings(self):
        for phrase, action, parameters in [
            ("stop", "stop", {}), ("speed up", "speed_up", {}),
            ("slow down", "slow_down", {}), ("take the next right", "turn", {"value": "right"}),
            ("go left", "turn", {"value": "left"}), ("go straight", "turn", {"value": "straight"})]:
            with self.subTest(phrase=phrase):
                result = self.session.chat(phrase)
                self.assertEqual(result.status, "preview_available")
                self.assertEqual((result.action, result.parameters), (action, parameters))
                self.assertTrue(result.offline_only)
                self.assertEqual(self.session.receiver.records, [])
                if action == "turn":
                    self.assertTrue(any("calibration" in p for p in result.prerequisites))

    def test_fixed_step_requires_explicit_clarification(self):
        for phrase in ["speed up a little", "a little faster", "a little slower", "slow down slightly", "speed up by 10 percent",
                       "slow down by 2 cm/s", "speed up by 0.2 m/s"]:
            with self.subTest(phrase=phrase):
                result = self.session.chat(phrase)
                self.assertEqual(result.status, "clarification_needed")
                self.assertIsNone(self.session.draft)
                with self.assertRaises(ValueError):
                    self.session.record()
                result = self.session.chat("use the standard step")
                self.assertEqual(result.status, "preview_available")
                self.assertIn("20%", result.explanation)
        self.session.chat("speed up a little")
        self.session.chat("no")
        self.assertIsNone(self.session.draft)
        self.assertNotEqual(self.session.chat("yes").status, "preview_available")

    def test_missing_details_and_followups(self):
        self.assertEqual(self.session.chat("turn").status, "clarification_needed")
        self.assertEqual(self.session.chat("right").parameters, {"value": "right"})
        self.assertEqual(self.session.chat("actually left").parameters, {"value": "left"})
        self.assertEqual(self.session.chat("the other direction").parameters, {"value": "right"})
        self.assertEqual(self.session.chat("speed up by 3").status, "clarification_needed")
        self.assertNotEqual(self.session.chat("yes").status, "preview_available")
        self.session.chat("speed up by 3")
        self.assertEqual(self.session.chat("percent").status, "clarification_needed")
        self.assertEqual(self.session.chat("yes").action, "speed_up")

    def test_unsupported_and_rejected_requests_clear_stale_draft(self):
        phrases = ["stop after two seconds", "stop after 30 cm", "stop at the red line",
                   "stop for five seconds", "reverse for two seconds", "undo that turn",
                   "interrupt all movement", "set speed to 10 cm/s", "don't turn left",
                   "if I say stop", "go left then stop", "bananas", "stop until the checkpoint"]
        for phrase in phrases:
            with self.subTest(phrase=phrase):
                self.session.chat("stop")
                result = self.session.chat(phrase)
                self.assertNotEqual(result.status, "preview_available")
                self.assertIsNone(self.session.draft)
                self.assertIsNone(result.action)
                with self.assertRaises(ValueError):
                    self.session.record()

    def test_conversation_cancel_and_clear(self):
        self.assertEqual(self.session.chat("hi").status, "conversation_only")
        self.assertIsNone(self.session.draft)
        self.session.chat("stop")
        self.session.chat("thanks")
        self.assertEqual(self.session.draft.action, "stop")
        self.session.record()
        self.session.chat("turn left")
        self.session.chat("cancel that")
        self.assertIsNone(self.session.draft)
        self.assertEqual(len(self.session.receiver.records), 1)
        self.session.clear()
        self.assertEqual(self.session.receiver.records, [])
        self.assertIsNone(self.session.interpreter.last_request)
        self.assertIsNone(self.session.draft)

    def test_records_are_independent_and_not_repeatable_without_new_draft(self):
        result = self.session.chat("turn right")
        result.parameters["value"] = "left"
        self.assertEqual(self.session.draft.parameters["value"], "right")
        record = self.session.record()
        record["preview"]["parameters"]["value"] = "straight"
        self.session.chat("actually left")
        self.session.record()
        records = self.session.receiver.records
        self.assertEqual([r["preview"]["parameters"]["value"] for r in records], ["right", "left"])
        records.clear()
        self.assertEqual(len(self.session.receiver.records), 2)
        with self.assertRaises(ValueError):
            self.session.record()

    def test_receiver_rejects_invalid_previews(self):
        receiver = FakeReceiver()
        for preview in [None, Preview("clarification_needed", "stop"),
                        Preview("preview_available", "reverse"),
                        Preview("preview_available", "stop", {"duration_s": 2}),
                        Preview("preview_available", "turn", {"value": "uturn"}),
                        Preview("preview_available", "turn", None),
                        Preview("preview_available", "stop", offline_only=False)]:
            with self.assertRaises(ValueError):
                receiver.record(preview)
        self.assertEqual(receiver.records, [])

    def test_greetings_preserve_unresolved_step_but_new_request_replaces_it(self):
        self.session.chat("slow down a little")
        self.session.chat("thanks")
        self.assertEqual(self.session.chat("yes").action, "slow_down")
        self.session.chat("slow down a little")
        self.session.chat("turn left")
        self.assertNotEqual(self.session.chat("yes").status, "preview_available")
        self.assertIsNone(self.session.draft)

    def test_source_imports_are_offline_only(self):
        folder = Path(__file__).resolve().parents[1] / "laptop"
        allowed = {"copy", "dataclasses", "offline_interpreter", "json", "tkinter", "command_preview"}
        for filename in ("command_preview.py", "command_preview_chat.py"):
            tree = ast.parse((folder/filename).read_text(encoding="utf-8"))
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    self.assertTrue(all(alias.name in allowed for alias in node.names))
                if isinstance(node, ast.ImportFrom):
                    self.assertIn(node.module, allowed)


class WindowTests(unittest.TestCase):
    @unittest.skipUnless(sys.platform == "win32", "Windows Tk lifecycle")
    def test_lifecycle_with_network_and_connected_code_blocked(self):
        original = builtins.__import__
        def guarded(name, *args, **kwargs):
            if name.split(".")[0] in {"rospy", "roslibpy", "chat_core", "duck2_chat", "command_gateway", "requests", "http"}:
                raise AssertionError("Forbidden import: " + name)
            return original(name, *args, **kwargs)
        with patch("builtins.__import__", side_effect=guarded), \
             patch.object(socket, "socket", side_effect=AssertionError("socket")), \
             patch.object(socket, "create_connection", side_effect=AssertionError("network")), \
             patch.object(urllib.request, "urlopen", side_effect=AssertionError("HTTP")), \
             patch.object(subprocess, "Popen", side_effect=AssertionError("process")):
            import tkinter as tk
            from command_preview_chat import CommandPreviewWindow
            root = tk.Tk()
            root.withdraw()
            window = CommandPreviewWindow(root)
            try:
                for phrase in ["hi", "take the next right", "actually left", "thanks"]:
                    window.message.set(phrase)
                    window.send()
                    root.update()
                self.assertFalse(window.record_button.instate(["disabled"]))
                window.record()
                self.assertEqual(window.session.receiver.records[0]["preview"]["parameters"], {"value": "left"})
                window.message.set("stop after two seconds")
                window.send()
                self.assertTrue(window.record_button.instate(["disabled"]))
                for phrase in ("duck", "go around it", "something in the way",
                               "don't avoid the duck", "the road is blocked"):
                    window.message.set(phrase)
                    window.send()
                    root.update()
                    self.assertTrue(window.record_button.instate(["disabled"]))
                    self.assertIsNone(window.session.draft)
                window.clear()
                self.assertEqual(window.session.receiver.records, [])
                self.assertEqual(window.history.get("1.0", "end").strip(), "")
            finally:
                window.close()
            self.assertIsNone(window.session.draft)


if __name__ == "__main__":
    unittest.main()
