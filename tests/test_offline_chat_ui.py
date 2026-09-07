"""Run on Windows Python with Tk: construct, use and close the actual window.

Network, child processes and imports of the connected companion are forbidden
throughout the window lifecycle. No robot, model or display automation is needed.
"""
import builtins
import json
import socket
import subprocess
import sys
import unittest
import urllib.request
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "laptop"))


class OfflineWindowTests(unittest.TestCase):
    @unittest.skipUnless(sys.platform == "win32", "Native Windows Tk test")
    def test_window_lifecycle_without_network_or_robot_code(self):
        original_import = builtins.__import__

        def isolated_import(name, *args, **kwargs):
            if name.split(".")[0] in {"chat_core", "duck2_chat", "rospy", "duckietown", "requests", "http"}:
                raise AssertionError("Offline UI tried to import " + name)
            return original_import(name, *args, **kwargs)

        with patch("builtins.__import__", side_effect=isolated_import), \
                patch.object(socket, "socket", side_effect=AssertionError("Network socket created")), \
                patch.object(socket, "create_connection", side_effect=AssertionError("Network connection attempted")), \
                patch.object(urllib.request, "urlopen", side_effect=AssertionError("HTTP attempted")), \
                patch.object(subprocess, "Popen", side_effect=AssertionError("Child process attempted")):
            import tkinter as tk
            from offline_chat import OfflineChatWindow
            root = tk.Tk()
            root.withdraw()
            try:
                window = OfflineChatWindow(root)
                root.update()
                for message in ("Hello", "Stop", "Interrupt all movement", "Reverse two seconds slowly", "Take the next right", "Actually left"):
                    window.message.set(message)
                    window.send()
                    root.update()
                    details = json.loads(window.details.get("1.0", "end"))
                    self.assertTrue(details["interpretation_only"])
                self.assertEqual(details["parameters"]["direction"], "left")
                window.clear()
                self.assertIsNone(window.interpreter.last_request)
                self.assertEqual(window.details.get("1.0", "end").strip(), "")
            finally:
                root.destroy()
            # Exercise the application's own close handler on a fresh instance.
            root = tk.Tk()
            root.withdraw()
            window = OfflineChatWindow(root)
            window.close()


if __name__ == "__main__":
    unittest.main()
