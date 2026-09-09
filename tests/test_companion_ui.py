"""Construct and use the combined Tk app while all network access is blocked."""

import socket
import sys
import unittest
import urllib.request
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "laptop"))


class CompanionWindowTests(unittest.TestCase):
    @unittest.skipUnless(sys.platform == "win32", "Native Windows Tk test")
    def test_open_plan_chat_and_close_without_network(self):
        with patch.object(socket, "socket", side_effect=AssertionError("Network socket created")), \
                patch.object(socket, "create_connection",
                             side_effect=AssertionError("Network connection attempted")), \
                patch.object(urllib.request, "urlopen",
                             side_effect=AssertionError("HTTP attempted")):
            import tkinter as tk
            from duck2_companion import CompanionWindow
            root = tk.Tk()
            root.geometry("900x650+10000+10000")
            window = CompanionWindow(root)
            try:
                root.update()
                self.assertEqual(root.title(), "duck2 companion")
                self.assertEqual(str(window.start_route_button.cget("state")), "disabled")
                self.assertEqual(
                    [window.notebook.tab(index, "text").strip()
                     for index in range(window.notebook.index("end"))],
                    ["1  Map & route", "2  Camera & chat"])
                window.notebook.select(window.workspace_tab)
                root.update()
                pane_width = window.workspace_panes.winfo_width()
                camera_width = window.workspace_panes.sashpos(0)
                self.assertEqual(str(window.workspace_panes.cget("orient")), "horizontal")
                self.assertGreater(pane_width, 420)
                self.assertGreater(camera_width, pane_width * 0.65)
                self.assertLess(camera_width, pane_width * 0.80)
                self.assertGreater(window.camera_tab.winfo_width(), pane_width * 0.65)
                self.assertGreater(window.camera_image.winfo_height(), 250)
                self.assertGreater(window.message_entry.winfo_reqheight(), 35)
                self.assertEqual(window.camera_state.get(),
                                 "Viewer stopped. No connection attempted.")
                window.choose_start("A->B")
                window.choose_destination("B->C")
                self.assertEqual(window.session.plan.route, ("A", "B", "C"))
                self.assertFalse(window.session.route_payload["execution_enabled"])
                window.message.set("Where are we going?")
                window.send_chat()
                self.assertIn("B->C", window.chat_log.get("1.0", "end"))
                window.clear_all()
                self.assertIsNone(window.session.plan)
            finally:
                window.close()


if __name__ == "__main__":
    unittest.main()
