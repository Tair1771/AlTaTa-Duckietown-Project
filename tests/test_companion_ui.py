"""Construct and use the combined Tk app while all network access is blocked."""

import socket
import sys
import unittest
import urllib.request
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "laptop"))


class CompanionWindowTests(unittest.TestCase):
    @unittest.skipUnless(sys.platform == "win32", "Native Windows Tk test")
    def test_bench_mode_uses_separate_loopback_endpoints_and_visible_label(self):
        with patch.object(socket, "socket", side_effect=AssertionError("Network attempted")):
            import tkinter as tk
            from duck2_companion import CompanionWindow
            root = tk.Tk()
            window = CompanionWindow(root, bench=True)
            try:
                root.geometry("1100x760+10000+10000")
                root.update()
                self.assertIn("SIMULATION", root.title())
                self.assertEqual(window.control_url.get(), "http://127.0.0.1:18765")
                self.assertEqual(window.camera_url.get(), "http://127.0.0.1:18766")
                self.assertIsNone(window.transport)
            finally:
                window.close()

    @unittest.skipUnless(sys.platform == "win32", "Native Windows Tk test")
    def test_display_scaling_keeps_map_and_route_actions_visible(self):
        with patch.object(socket, "socket", side_effect=AssertionError("Network attempted")):
            import tkinter as tk
            from duck2_companion import CompanionWindow
            for scaling in (1.33, 2.0, 2.67):
                with self.subTest(scaling=scaling):
                    root = tk.Tk()
                    original_scaling = root.tk.call("tk", "scaling")
                    root.tk.call("tk", "scaling", scaling)
                    window = CompanionWindow(root)
                    try:
                        root.geometry("850x600+10000+10000")
                        root.update()
                        window.choose_start("A->B")
                        self.assertEqual(window.selection_mode.get(), "destination")
                        window.choose_destination("B->C")
                        window.fit_map_button.invoke()
                        root.update()
                        self.assertEqual(window.session.plan.route, ("A", "B", "C"))
                        x0, y0, x1, y1 = window.canvas.bbox("all")
                        self.assertGreaterEqual(min(x0, y0), 0)
                        self.assertLessEqual(x1, window.canvas.winfo_width())
                        self.assertLessEqual(y1, window.canvas.winfo_height())
                        for widget in (window.map_position_check, window.start_route_button,
                                       window.map_stop_button, window.fit_map_button):
                            self.assertLessEqual(widget.winfo_rooty() + widget.winfo_height(),
                                                 root.winfo_rooty() + root.winfo_height())
                            self.assertLessEqual(widget.winfo_rootx() + widget.winfo_width(),
                                                 root.winfo_rootx() + root.winfo_width())
                    finally:
                        root.tk.call("tk", "scaling", original_scaling)
                        window.close()
                        self.assertFalse(window._after_calls)

    @unittest.skipUnless(sys.platform == "win32", "Native Windows Tk test")
    def test_map_fits_and_selection_tracks_resizing(self):
        with patch.object(socket, "socket", side_effect=AssertionError("Network attempted")):
            import tkinter as tk
            from duck2_companion import CompanionWindow, red_line_point, directed_points
            from route_planner import RED_LINE_APPROACHES
            root = tk.Tk()
            window = CompanionWindow(root)
            try:
                for size in ("850x600", "1100x760", "1600x900", "900x650"):
                    with self.subTest(size=size):
                        root.geometry(size + "+10000+10000")
                        root.update()
                        x0, y0, x1, y1 = window.canvas.bbox("all")
                        self.assertGreaterEqual(x0, 0)
                        self.assertGreaterEqual(y0, 0)
                        self.assertLessEqual(x1, window.canvas.winfo_width())
                        self.assertLessEqual(y1, window.canvas.winfo_height())
                        for widget in (window.map_position_check, window.start_route_button,
                                       window.map_stop_button, window.route_text):
                            self.assertTrue(widget.winfo_ismapped())
                            self.assertLessEqual(widget.winfo_rooty() + widget.winfo_height(),
                                                 root.winfo_rooty() + root.winfo_height())
                        window.selection_mode.set("start")
                        points = directed_points("A", "B")
                        scale, ox, oy = window._map_transform
                        window.map_click(SimpleNamespace(
                            x=(points[0]+points[2])/2*scale+ox,
                            y=(points[1]+points[3])/2*scale+oy))
                        self.assertEqual(window.session.start_approach, "A->B")
                        window.selection_mode.set("destination")
                        for value in RED_LINE_APPROACHES:
                            scale, ox, oy = window._map_transform
                            x, y = red_line_point(value)
                            window.map_click(SimpleNamespace(x=x*scale+ox, y=y*scale+oy))
                            self.assertEqual(window.session.destination_approach, value)
                        window.notebook.select(window.workspace_tab)
                        root.update()
                        for widget in (window.start_camera_button, window.control_start_button,
                                       window.control_stop_button):
                            self.assertTrue(widget.winfo_ismapped())
                            self.assertGreaterEqual(widget.winfo_rootx(), root.winfo_rootx())
                            self.assertLessEqual(widget.winfo_rootx() + widget.winfo_width(),
                                                 root.winfo_rootx() + root.winfo_width())
                            self.assertLessEqual(widget.winfo_rooty() + widget.winfo_height(),
                                                 root.winfo_rooty() + root.winfo_height())
                        window.control_notebook.select(1)
                        root.update()
                        self.assertGreater(window.chat_log.winfo_height(), 60)
                        self.assertGreater(window.message_entry.winfo_height(), 35)
                        self.assertTrue(window.control_stop_button.winfo_ismapped())
                        window.control_notebook.select(0)
                        window.notebook.select(window.map_tab)
                        root.update()
                window.control_connected = True
                window.choose_start("A->B")
                window.choose_destination("B->C")
                window.position_confirmed.set(True)
                window._update_route_button()
                self.assertEqual(str(window.start_route_button.cget("state")), "normal")
                window.choose_start("B->C")
                self.assertFalse(window.position_confirmed.get())
                self.assertEqual(str(window.start_route_button.cget("state")), "disabled")
            finally:
                window.close()

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
                    ["1  Map & route", "2  Camera & status"])
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
                self.assertEqual(window.camera_state.get(),
                                 "Viewer stopped. No connection attempted.")
                window.choose_start("A->B")
                window.choose_destination("B->C")
                self.assertEqual(window.session.plan.route, ("A", "B", "C"))
                self.assertFalse(window.session.route_payload["execution_enabled"])
                self.assertTrue(hasattr(window, "message_entry"))
                self.assertTrue(window.live_chat_enabled.get())
                window.control_notebook.select(1)
                root.update()
                self.assertTrue(window.message_entry.winfo_ismapped())
                self.assertGreater(window.message_entry.winfo_height(), 35)
                self.assertIn("Start selected route",
                              window.start_route_button.cget("text"))
                window.clear_all()
                self.assertIsNone(window.session.plan)
            finally:
                window.close()

    @unittest.skipUnless(sys.platform == "win32", "Native Windows Tk test")
    def test_stop_is_sent_even_when_a_chat_operation_holds_the_lock(self):
        import threading
        import tkinter as tk
        from duck2_companion import CompanionWindow
        with patch.object(socket, "socket", side_effect=AssertionError("No robot access")):
            root = tk.Tk()
            window = CompanionWindow(root)
            sent = threading.Event()
            window.transport = SimpleNamespace(
                send=lambda *args, **kwargs: (sent.set() or {"accepted": True}),
                status=lambda: {"state": "route_complete"})
            window.control_operation_lock.acquire()
            try:
                window.stop_robot()
                self.assertTrue(sent.wait(1), "Stop must not wait for the chat operation")
                self.assertTrue(window.stop_requested)
            finally:
                window.control_operation_lock.release()
                window.close()


if __name__ == "__main__":
    unittest.main()
