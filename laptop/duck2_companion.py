#!/usr/bin/env python3
"""Windows route controller and camera viewer for duck2."""

import io
import math
import os
import queue
import threading
import tkinter as tk
from tkinter import ttk

try:
    from PIL import Image, ImageEnhance, ImageFilter, ImageOps, ImageTk
except ImportError:  # Keep the app usable with a standard-library-only Python.
    Image = None
    ImageEnhance = None
    ImageFilter = None
    ImageOps = None
    ImageTk = None

from camera_client import CameraClient, CameraPoller
from chat_core import RobotTransport
from companion_core import OfflineCompanionSession
from route_planner import (MAP_NODES, MAP_ROADS, RED_LINE_APPROACHES,
                           approach_id, parse_approach)
from route_control import start_route
from live_chat import LiveChatSession, interpret_live


COLORS = {
    "background": "#edf3f0",
    "card": "#ffffff",
    "ink": "#17231f",
    "muted": "#5f7069",
    "header": "#102d25",
    "header_muted": "#b8cbc3",
    "green": "#177a55",
    "green_hover": "#116443",
    "green_soft": "#dff3e9",
    "map": "#15221e",
    "road": "#3f504a",
    "lane": "#d9c45d",
    "border": "#d6e1dc",
}


def _pairs(points):
    return list(zip(points[::2], points[1::2]))


def map_point(x, y):
    """Fit the schematic course comfortably inside the default map panel."""
    return 22.0 + x * 1.18, 9.0 + y * 0.88


def display_points(points):
    return tuple(coordinate for x, y in _pairs(points)
                 for coordinate in map_point(x, y))


def directed_points(previous, junction, offset=6.0):
    points = _pairs(MAP_ROADS.get((previous, junction), ()))
    if not points:
        points = list(reversed(_pairs(MAP_ROADS[(junction, previous)])))
    points = [map_point(x, y) for x, y in points]
    dx, dy = points[-1][0] - points[0][0], points[-1][1] - points[0][1]
    length = math.hypot(dx, dy) or 1.0
    # Screen y grows downward. This offset represents the traveller's right lane.
    ox, oy = -dy / length * offset, dx / length * offset
    return tuple(coordinate for x, y in points for coordinate in (x + ox, y + oy))


def red_line_point(approach):
    previous, junction = parse_approach(approach)
    points = _pairs(directed_points(previous, junction))
    end, before = points[-1], points[-2]
    dx, dy = end[0] - before[0], end[1] - before[1]
    length = math.hypot(dx, dy) or 1.0
    return end[0] - dx / length * 21, end[1] - dy / length * 21


def point_segment_distance(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    if dx == dy == 0:
        return math.hypot(px - ax, py - ay)
    amount = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) /
                              float(dx * dx + dy * dy)))
    return math.hypot(px - (ax + amount * dx), py - (ay + amount * dy))


class CompanionWindow:
    def __init__(self, root):
        self.root = root
        self.session = OfflineCompanionSession()
        self.events = queue.Queue()
        self.transport = None
        self.control_connected = False
        self.control_polling = False
        self.heartbeat_pending = False
        self.live_session = None
        self.stop_requested = True
        self.live_chat_enabled = tk.BooleanVar(value=True)
        self.control_operation_lock = threading.Lock()
        self.start_generation = 0
        self.route_start_pending = False
        self.position_confirmed = tk.BooleanVar(value=False)
        self._map_transform = (1.0, 0.0, 0.0)
        self.poller = None
        self.photo = None
        self.closed = False
        self._after_calls = set()
        self._workspace_split_initialized = False
        root.title("duck2 companion")
        root.geometry("1100x760")
        # Above 150% display scaling the system fonts need more room too.
        # Keep route confirmation and Stop visible instead of shrinking their
        # container below the controls' actual requested size.
        large_text = max(1.0, float(root.tk.call("tk", "scaling")) / 2.0)
        root.minsize(round(850 * large_text), round(600 * large_text))
        root.configure(background=COLORS["background"])
        self._configure_style()

        outer = ttk.Frame(root, padding=(18, 16, 18, 16), style="App.TFrame")
        outer.pack(fill="both", expand=True)
        header = ttk.Frame(outer, padding=(14, 8), style="Header.TFrame")
        header.pack(fill="x", pady=(0, 10))
        heading = ttk.Frame(header, style="Header.TFrame")
        heading.pack(side="left", fill="x", expand=True)
        ttk.Label(heading, text="duck2 companion", style="HeaderTitle.TLabel").pack(anchor="w")
        self.connection_badge = tk.StringVar(value="●  DISCONNECTED")
        ttk.Label(header, textvariable=self.connection_badge,
                  style="Safety.TLabel").pack(side="right", padx=(12, 0))

        self.status = tk.StringVar(value="Select the directed starting lane and a red-line destination.")
        status_bar = ttk.Frame(outer, padding=(13, 9), style="Status.TFrame")
        status_bar.pack(fill="x", pady=(0, 10))
        ttk.Label(status_bar, text="STATUS", style="StatusCaption.TLabel").pack(side="left")
        status_label = ttk.Label(status_bar, textvariable=self.status, style="StatusText.TLabel",
                                 wraplength=900)
        status_label.pack(side="left", padx=(12, 0), fill="x", expand=True)
        self._wrap_label(status_label)

        self.notebook = ttk.Notebook(outer, style="App.TNotebook")
        self.notebook.pack(fill="both", expand=True)
        self.map_tab = ttk.Frame(self.notebook, padding=12, style="App.TFrame")
        self.workspace_tab = ttk.Frame(self.notebook, padding=10, style="App.TFrame")
        self.notebook.add(self.map_tab, text="  1  Map & route  ")
        self.notebook.add(self.workspace_tab, text="  2  Camera & status  ")
        self.notebook.bind("<<NotebookTabChanged>>", self._notebook_tab_changed)
        self.workspace_panes = ttk.Panedwindow(self.workspace_tab, orient="horizontal")
        self.workspace_panes.pack(fill="both", expand=True)
        self.camera_tab = ttk.Frame(self.workspace_panes, padding=(8, 6), style="App.TFrame")
        self.chat_tab = ttk.Frame(self.workspace_panes, padding=(8, 6), style="App.TFrame")
        self.workspace_panes.add(self.camera_tab, weight=7)
        self.workspace_panes.add(self.chat_tab, weight=1)
        self.workspace_panes.bind("<Configure>", self._keep_control_visible)
        self._build_map()
        self._build_camera()
        self._build_control()
        root.protocol("WM_DELETE_WINDOW", self.close)
        self._schedule(100, self._poll_camera)
        self._schedule(100, self._drain_control)
        self._schedule(500, self._poll_control)
        self._schedule("idle", self._set_workspace_split)

    def _schedule(self, delay, callback):
        def run():
            self._after_calls.discard(identifier)
            if not self.closed:
                callback()
        identifier = (self.root.after_idle(run) if delay == "idle"
                      else self.root.after(delay, run))
        self._after_calls.add(identifier)

    def _configure_style(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", font=("Segoe UI", 10), foreground=COLORS["ink"])
        style.configure("App.TFrame", background=COLORS["background"])
        style.configure("Header.TFrame", background=COLORS["header"])
        style.configure("HeaderTitle.TLabel", background=COLORS["header"],
                        foreground="#ffffff", font=("Segoe UI", 20, "bold"))
        style.configure("HeaderSubtitle.TLabel", background=COLORS["header"],
                        foreground=COLORS["header_muted"])
        style.configure("Safety.TLabel", background="#1c493c", foreground="#c9f6df",
                        padding=(12, 7), font=("Segoe UI", 9, "bold"))
        style.configure("Status.TFrame", background=COLORS["green_soft"])
        style.configure("StatusCaption.TLabel", background=COLORS["green_soft"],
                        foreground=COLORS["green"], font=("Segoe UI", 9, "bold"))
        style.configure("StatusText.TLabel", background=COLORS["green_soft"],
                        foreground=COLORS["ink"])
        style.configure("Card.TFrame", background=COLORS["card"])
        style.configure("Card.TLabelframe", background=COLORS["card"],
                        bordercolor=COLORS["border"], relief="solid")
        style.configure("Card.TLabelframe.Label", background=COLORS["card"],
                        foreground=COLORS["ink"], font=("Segoe UI", 11, "bold"))
        style.configure("Section.TLabel", background=COLORS["background"],
                        foreground=COLORS["ink"], font=("Segoe UI", 15, "bold"))
        style.configure("Hint.TLabel", background=COLORS["background"],
                        foreground=COLORS["muted"])
        style.configure("CardHint.TLabel", background=COLORS["card"],
                        foreground=COLORS["muted"])
        style.configure("Primary.TButton", background=COLORS["green"],
                        foreground="#ffffff", padding=(12, 8), font=("Segoe UI", 10, "bold"))
        style.map("Primary.TButton", background=[("active", COLORS["green_hover"]),
                                                  ("disabled", "#a5b5ae")])
        style.configure("Secondary.TButton", padding=(10, 7))
        style.configure("ChatInput.TEntry", padding=(10, 12),
                        fieldbackground=COLORS["card"])
        style.configure("App.TNotebook", background=COLORS["background"], borderwidth=0)
        style.configure("App.TNotebook.Tab", padding=(14, 9), font=("Segoe UI", 10, "bold"))
        style.map("App.TNotebook.Tab",
                  background=[("selected", COLORS["card"]), ("!selected", "#dce7e2")],
                  foreground=[("selected", COLORS["green"]), ("!selected", COLORS["muted"])])

    def _set_workspace_split(self):
        """Give the camera most of the width once the workspace is visible."""
        if self.closed or self._workspace_split_initialized:
            return
        self.root.update_idletasks()
        width = self.workspace_panes.winfo_width()
        if width > 420:
            self.workspace_panes.sashpos(0, min(int(width * 0.72), width - 290))
            self._workspace_split_initialized = True

    def _keep_control_visible(self, event):
        if self._workspace_split_initialized and event.width > 420:
            current = self.workspace_panes.sashpos(0)
            self.workspace_panes.sashpos(0, min(current, event.width - 290))

    def _notebook_tab_changed(self, event=None):
        """Set the initial split only after the shared workspace is visible."""
        if self.notebook.select() == str(self.workspace_tab):
            self._schedule("idle", self._set_workspace_split)

    def _build_map(self):
        controls = ttk.Frame(self.map_tab, style="App.TFrame")
        controls.pack(fill="x", pady=(0, 8))
        self.selection_mode = tk.StringVar(value="start")
        values = list(RED_LINE_APPROACHES)
        self.start_value = tk.StringVar()
        self.destination_value = tk.StringVar()

        start_card = ttk.Frame(controls, style="App.TFrame")
        controls.columnconfigure((0, 1), weight=1, uniform="selector")
        start_card.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        ttk.Radiobutton(start_card, text="1  Start lane", value="start",
                        variable=self.selection_mode).pack(side="left")
        start = ttk.Combobox(start_card, textvariable=self.start_value, values=values,
                             state="readonly", width=10)
        start.pack(side="right", padx=(6, 0))

        destination_card = ttk.Frame(controls, style="App.TFrame")
        destination_card.grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Radiobutton(destination_card, text="2  Destination", value="destination",
                        variable=self.selection_mode).pack(side="left")
        destination = ttk.Combobox(destination_card, textvariable=self.destination_value,
                                   values=values, state="readonly", width=10)
        destination.pack(side="right", padx=(6, 0))
        ttk.Button(controls, text="Clear route", command=self.clear_route,
                   style="Secondary.TButton").grid(row=0, column=2, padx=(6, 0))
        start.bind("<<ComboboxSelected>>", lambda event: self.choose_start(self.start_value.get()))
        destination.bind("<<ComboboxSelected>>",
                         lambda event: self.choose_destination(self.destination_value.get()))

        body = ttk.Frame(self.map_tab, style="App.TFrame")
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=0, minsize=250)
        body.rowconfigure(0, weight=1)
        map_panel = ttk.Frame(body, style="App.TFrame")
        map_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        map_footer = ttk.Frame(map_panel, style="App.TFrame")
        map_footer.pack(side="bottom", fill="x", pady=(5, 0))
        self.fit_map_button = ttk.Button(map_footer, text="Fit map", command=self.draw_map)
        self.fit_map_button.pack(side="right")
        legend = ttk.Label(map_footer, text="Blue: start • Green: route\nClick a red marker for the destination.",
                           style="Hint.TLabel")
        legend.pack(side="left", fill="x", expand=True)
        self._wrap_label(legend)
        self.canvas = tk.Canvas(map_panel, width=1, height=1, bg=COLORS["map"],
                                highlightthickness=1, highlightbackground=COLORS["border"])
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Button-1>", self.map_click)
        self.canvas.bind("<Configure>", lambda event: self.draw_map())
        side = ttk.LabelFrame(body, text="Route preview", padding=10,
                              style="Card.TLabelframe")
        side.grid(row=0, column=1, sticky="nsew")
        side.rowconfigure(0, weight=1)
        side.columnconfigure(0, weight=1)
        self.route_text = tk.Text(side, width=26, height=1, wrap="word", state="disabled",
                                  relief="flat", borderwidth=0, background=COLORS["card"],
                                  foreground=COLORS["ink"], font=("Segoe UI", 11),
                                  padx=4, pady=4)
        self.route_text.grid(row=0, column=0, sticky="nsew", pady=6)
        scrollbar = ttk.Scrollbar(side, orient="vertical", command=self.route_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns", pady=6)
        self.route_text.configure(yscrollcommand=scrollbar.set)
        self.map_position_check = ttk.Checkbutton(
            side, text="On the selected starting lane\nand outside the intersection",
            variable=self.position_confirmed, command=self._update_route_button)
        self.map_position_check.grid(row=1, column=0, columnspan=2, sticky="w", pady=4)
        self.start_route_button = ttk.Button(
            side, text="Start selected route", command=self.start_selected_route,
            state="disabled", style="Primary.TButton")
        self.start_route_button.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(6, 8))
        self.map_stop_button = self._stop_button(side)
        self.map_stop_button.grid(row=3, column=0, columnspan=2, sticky="ew")
        self.draw_map()
        self.refresh_route()

    def _build_camera(self):
        camera_heading = ttk.Frame(self.camera_tab, style="App.TFrame")
        camera_heading.pack(fill="x", pady=(0, 6))
        ttk.Label(camera_heading, text="Live camera",
                  style="Section.TLabel").pack(side="left")

        top = ttk.LabelFrame(self.camera_tab, text="Camera connection", padding=(10, 7),
                             style="Card.TLabelframe")
        top.pack(fill="x", pady=(0, 7))
        connection_row = ttk.Frame(top, style="Card.TFrame")
        connection_row.pack(fill="x")
        self.camera_url = tk.StringVar(value="http://127.0.0.1:8766")
        ttk.Label(connection_row, text="Local address").pack(side="left")
        ttk.Entry(connection_row, textvariable=self.camera_url, width=28).pack(
            side="left", padx=8, fill="x", expand=True)
        camera_actions = ttk.Frame(top, style="Card.TFrame")
        camera_actions.pack(fill="x", pady=(6, 0))
        self.start_camera_button = ttk.Button(camera_actions, text="Start viewing",
                                              command=self.start_camera,
                                              style="Primary.TButton")
        self.start_camera_button.pack(side="left")
        ttk.Button(camera_actions, text="Stop viewing", command=self.stop_camera,
                   style="Secondary.TButton").pack(side="left", padx=(6, 12))
        view_row = ttk.Frame(top, style="Card.TFrame")
        view_row.pack(fill="x", pady=(6, 0))
        ttk.Label(view_row, text="View").pack(side="left", padx=(0, 3))
        self.camera_view = tk.StringVar(value="normal")
        for view in CameraClient.VIEWS:
            ttk.Radiobutton(view_row, text=view.title(), value=view,
                            variable=self.camera_view,
                            command=self.change_camera_view).pack(side="left", padx=2)
        self.camera_state = tk.StringVar(value="Viewer stopped. No connection attempted.")
        camera_status = ttk.Frame(self.camera_tab, padding=(10, 6), style="Status.TFrame")
        camera_status.pack(fill="x", pady=(0, 7))
        ttk.Label(camera_status, text="CAMERA", style="StatusCaption.TLabel").pack(side="left")
        camera_label = ttk.Label(camera_status, textvariable=self.camera_state,
                                 style="StatusText.TLabel", wraplength=400)
        camera_label.pack(side="left", padx=(12, 0), fill="x", expand=True)
        self._wrap_label(camera_label)
        self.camera_image = ttk.Label(
            self.camera_tab, text="Select Start viewing to open the camera.",
            anchor="center", background="#16231f", foreground="#c9d8d1")
        self.camera_image.pack(fill="both", expand=True)

    def _build_control(self):
        self.chat_tab.columnconfigure(0, weight=1)
        self.chat_tab.rowconfigure(1, weight=1)
        chat_header = ttk.Frame(self.chat_tab, style="App.TFrame")
        chat_header.grid(row=0, column=0, sticky="ew", pady=(0, 5))
        ttk.Label(chat_header, text="Route control", style="Section.TLabel").pack(anchor="w")
        ttk.Label(chat_header, text="Map routes • local live chat",
                  style="Hint.TLabel").pack(anchor="w")
        self.control_notebook = ttk.Notebook(self.chat_tab)
        self.control_notebook.grid(row=1, column=0, sticky="nsew")
        connection = ttk.LabelFrame(self.control_notebook, text="Connection", padding=10,
                                    style="Card.TLabelframe")
        self.control_notebook.add(connection, text="Connection")
        chat = ttk.Frame(self.control_notebook, padding=5)
        self.control_notebook.add(chat, text="Live chat")
        chat.columnconfigure(0, weight=1)
        chat.rowconfigure(1, weight=1)
        self.queue_summary = tk.StringVar(value="Start a live-chat run to give instructions.")
        queue_label = ttk.Label(chat, textvariable=self.queue_summary, wraplength=230)
        queue_label.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 4))
        self._wrap_label(queue_label)
        self.chat_log = tk.Text(chat, height=1, width=1, wrap="word", state="disabled",
                                font=("Segoe UI", 10), relief="flat")
        self.chat_log.grid(row=1, column=0, sticky="nsew")
        chat_scroll = ttk.Scrollbar(chat, command=self.chat_log.yview)
        chat_scroll.grid(row=1, column=1, sticky="ns")
        self.chat_log.configure(yscrollcommand=chat_scroll.set)
        self.message_entry = tk.Text(chat, height=3, width=1, wrap="word", font=("Segoe UI", 10))
        self.message_entry.grid(row=2, column=0, columnspan=2, sticky="ew", pady=5)
        self.message_entry.bind("<Return>", self.send_chat)
        self.message_entry.bind("<Shift-Return>", lambda event: None)
        ttk.Button(chat, text="Send message", command=self.send_chat).grid(
            row=3, column=0, columnspan=2, sticky="ew")
        self._append_chat("Duck2", "Ready for your instructions.")
        self.control_url = tk.StringVar(value="http://127.0.0.1:8765")
        ttk.Entry(connection, textvariable=self.control_url, width=24).pack(
            fill="x", pady=(0, 7))
        ttk.Button(connection, text="Connect to duck2", command=self.connect_control,
                   style="Primary.TButton").pack(fill="x")
        self.robot_state = tk.StringVar(value="Start lane-continuous and its SSH tunnel, then connect.")
        robot_label = ttk.Label(connection, textvariable=self.robot_state, wraplength=220,
                               style="CardHint.TLabel", justify="left")
        robot_label.pack(fill="x", pady=(10, 0))
        self._wrap_label(robot_label)

        actions = ttk.Frame(self.chat_tab, style="App.TFrame")
        actions.grid(row=2, column=0, sticky="sew", pady=(10, 0))
        self.route_actions = ttk.Frame(actions, style="App.TFrame")
        self.route_actions.pack(fill="x")
        ttk.Checkbutton(self.route_actions,
                        text="Live chat (ask when queue ends)",
                        variable=self.live_chat_enabled).pack(anchor="w", pady=(0, 5))
        ttk.Checkbutton(self.route_actions,
                        text="On the selected starting lane\nand outside the intersection",
                        variable=self.position_confirmed,
                        command=self._update_route_button).pack(anchor="w", pady=(0, 8))
        self.control_start_button = ttk.Button(
            self.route_actions, text="Start selected route", command=self.start_selected_route,
            state="disabled", style="Primary.TButton")
        self.control_start_button.pack(fill="x", pady=(0, 8))
        self.control_stop_button = self._stop_button(actions)
        self.control_stop_button.pack(fill="x")
        stop_hint = ttk.Label(actions,
                  text=("Live chat waits at red lines for instructions (60s limit). "
                        "Map-only mode ends at its destination. Closing requests Stop."),
                  style="Hint.TLabel", wraplength=220, justify="left")
        stop_hint.pack(fill="x", pady=(8, 0))
        self._wrap_label(stop_hint)
        self.control_stop_hint = stop_hint
        self.control_notebook.bind("<<NotebookTabChanged>>", self._show_control_tab)

    def _show_control_tab(self, event=None):
        # Starting controls remain on the map and Connection tabs. Chat gets
        # the vertical space while Stop stays visible beside it at every size.
        if self.control_notebook.index("current") == 1:
            self.route_actions.pack_forget()
            self.control_stop_hint.pack_forget()
        else:
            self.route_actions.pack(fill="x", before=self.control_stop_button)
            self.control_stop_hint.pack(fill="x", pady=(8, 0))

    def _stop_button(self, parent):
        return tk.Button(parent, text="STOP DUCK2", command=self.stop_robot,
                         bg="#b72f35", activebackground="#92242a", fg="white",
                         activeforeground="white", font=("Segoe UI", 11, "bold"),
                         padx=12, pady=9)

    @staticmethod
    def _wrap_label(label):
        label.bind("<Configure>", lambda event: label.configure(
            wraplength=max(100, event.width - 4)))

    def _replace(self, widget, text):
        widget.configure(state="normal")
        widget.delete("1.0", "end")
        widget.insert("end", text)
        widget.configure(state="disabled")

    def choose_start(self, approach):
        try:
            reply = self.session.select_start(approach)
            self.position_confirmed.set(False)
            self.start_value.set(approach)
            if self.session.destination_approach is None:
                self.selection_mode.set("destination")
            self.status.set(reply.explanation)
        except ValueError as error:
            self.status.set(str(error))
        self.refresh_route()

    def choose_destination(self, approach):
        try:
            reply = self.session.select_destination(approach)
            self.position_confirmed.set(False)
            self.destination_value.set(approach)
            self.status.set(reply.explanation)
        except ValueError as error:
            self.status.set(str(error))
        self.refresh_route()

    def map_click(self, event):
        scale, offset_x, offset_y = self._map_transform
        x, y = (event.x - offset_x) / scale, (event.y - offset_y) / scale
        if self.selection_mode.get() == "destination":
            candidates = [(math.hypot(x - red_line_point(value)[0],
                                      y - red_line_point(value)[1]), value)
                          for value in RED_LINE_APPROACHES]
            distance, value = min(candidates)
            if distance <= 15:
                self.choose_destination(value)
            else:
                self.status.set("Destinations can only be selected at a red marker.")
            return
        candidates = []
        for value in RED_LINE_APPROACHES:
            points = _pairs(directed_points(*parse_approach(value)))
            distance = min(point_segment_distance(x, y, *first, *second)
                           for first, second in zip(points, points[1:]))
            candidates.append((distance, value))
        distance, value = min(candidates)
        if distance <= 15:
            self.choose_start(value)
        else:
            self.status.set("Click one of the arrowed right-lane paths to select the start direction.")

    def draw_map(self):
        self.canvas.delete("all")
        for points in MAP_ROADS.values():
            rendered = display_points(points)
            self.canvas.create_line(*rendered, fill=COLORS["road"], width=27,
                                    joinstyle="round")
            self.canvas.create_line(*rendered, fill=COLORS["lane"], width=2, dash=(6, 6),
                                    joinstyle="round")
        route_segments = set()
        if self.session.plan:
            route_segments = {approach_id(a, b) for a, b in
                              zip(self.session.plan.route, self.session.plan.route[1:])}
        for value in RED_LINE_APPROACHES:
            previous, junction = parse_approach(value)
            points = directed_points(previous, junction)
            color = "#63dea1" if value in route_segments else "#aebbb6"
            if value == self.session.start_approach:
                color = "#69b8ff"
            self.canvas.create_line(*points, fill=color, width=3, arrow="last",
                                    arrowshape=(8, 10, 4), joinstyle="round")
            x, y = red_line_point(value)
            selected = value == self.session.destination_approach
            self.canvas.create_oval(x-6, y-6, x+6, y+6,
                                    fill="#ff5252" if selected else "#d92f2f",
                                    outline="#ffffff" if selected else "")
        for name, (x, y) in MAP_NODES.items():
            x, y = map_point(x, y)
            self.canvas.create_oval(x-13, y-13, x+13, y+13, fill="#f5f7f1", outline="")
            self.canvas.create_text(x, y, text=name, fill="#17231f",
                                    font=("Segoe UI", 11, "bold"))
        # Fit the entire course in the actual viewport. Keep the inverse
        # transform for clicks so red markers and directed lanes remain exact.
        x0, y0, x1, y1 = self.canvas.bbox("all")
        width, height = max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())
        scale = max(0.001, min(max(1, width-40)/(x1-x0), max(1, height-40)/(y1-y0)))
        offset_x = (width - (x1-x0)*scale)/2 - x0*scale
        offset_y = (height - (y1-y0)*scale)/2 - y0*scale
        self.canvas.scale("all", 0, 0, scale, scale)
        self.canvas.move("all", offset_x, offset_y)
        # Tk scales coordinates only, not stroke widths or fonts. Leaving
        # those at their original size clips edges at small/high-DPI sizes.
        for item in self.canvas.find_all():
            kind = self.canvas.type(item)
            if kind == "line":
                self.canvas.itemconfigure(item, width=max(1, float(self.canvas.itemcget(item, "width")) * scale))
                if self.canvas.itemcget(item, "arrow") != "none":
                    self.canvas.itemconfigure(item, arrowshape=tuple(max(2, n * scale) for n in (8, 10, 4)))
            elif kind == "text":
                self.canvas.itemconfigure(item, font=("Segoe UI", -max(9, round(15 * scale)), "bold"))
        self._map_transform = (scale, offset_x, offset_y)

    def refresh_route(self):
        self.draw_map()
        plan = self.session.plan
        if not plan:
            text = "Start: %s\nDestination: %s\n\nSelect both to calculate a route." % (
                self.session.start_approach or "—", self.session.destination_approach or "—")
        else:
            steps = []
            for index, turn in enumerate(plan.turns):
                steps.append("At %s: %s" % (plan.route[index + 1], turn.upper()))
            readiness = ("READY TO SEND AFTER PLACEMENT CONFIRMATION"
                         if self.control_connected else "LOCAL ROUTE — connect to send")
            text = ("Start lane: %s\nDestination red line: %s\n\nPath: %s\n"
                    "Junctions crossed: %d\n\n%s\n\n%s"
                    % (plan.start_approach, plan.destination_approach,
                       " → ".join(plan.route), plan.junction_count,
                       "\n".join(steps) if steps else "Already on the destination approach.",
                       readiness))
        self._replace(self.route_text, text)
        self._update_route_button()

    def _update_route_button(self):
        enabled = bool(self.session.plan and self.session.draft_valid
                       and self.control_connected
                       and not self.route_start_pending
                       and not (self.live_session is not None and self.live_session.active)
                       and getattr(self, "position_confirmed", None)
                       and self.position_confirmed.get())
        state = "normal" if enabled else "disabled"
        if hasattr(self, "start_route_button"):
            self.start_route_button.configure(state=state)
        if hasattr(self, "control_start_button"):
            self.control_start_button.configure(state=state)

    def clear_route(self):
        self.session.destination_approach = self.session.plan = None
        self.session.draft_valid = False
        self.destination_value.set("")
        self.position_confirmed.set(False)
        self.status.set("Route selection cleared. Press Stop separately if duck2 is moving.")
        self.refresh_route()

    def clear_all(self):
        self.session.reset()
        self.start_value.set("")
        self.destination_value.set("")
        self.position_confirmed.set(False)
        self.status.set("Select the directed starting lane and a red-line destination.")
        self.refresh_route()

    def _background(self, operation, kind):
        def work():
            try:
                self.events.put((kind, operation()))
            except Exception as error:
                self.events.put(("route_start_error" if kind == "route_started" else "control_error", str(error)))
            finally:
                if kind == "control_status":
                    self.control_polling = False
        threading.Thread(target=work, daemon=True).start()

    def connect_control(self):
        if self.route_start_pending:
            self.status.set("Wait for Start to finish, or use STOP DUCK2 before reconnecting.")
            return
        if self.live_session is not None and self.live_session.active:
            self.status.set("End the current run before replacing its connection.")
            return
        try:
            self.transport = RobotTransport(
                self.control_url.get(), os.environ.get("DUCK2_CONTROL_TOKEN", ""))
        except ValueError as error:
            self.robot_state.set(str(error))
            return
        self.robot_state.set("Connecting…")
        self.control_polling = True
        self._background(self.transport.poll_status, "control_status")

    def _format_robot_status(self, value):
        detail = value.get("fault") or value.get("stop_reason") or ""
        route = " → ".join(value.get("route") or []) or "none"
        wheels = value.get("wheel_speeds")
        if (isinstance(wheels, (list, tuple)) and len(wheels) == 2
                and all(isinstance(wheel, (int, float)) for wheel in wheels)):
            wheels = "L %.3f / R %.3f" % tuple(wheels)
        else:
            wheels = "unavailable"
        return ("State: %s\nNext junction: %s\nRoute: %s\nRequested wheels: %s%s" % (
            value.get("state", "unknown"), value.get("next_junction") or "—",
            route, wheels,
            ("\nStop reason: " + detail) if detail else ""))

    def _poll_control(self):
        if self.closed:
            return
        transport = self.transport
        if transport is not None and not self.heartbeat_pending:
            self.heartbeat_pending = True
            def heartbeat():
                try:
                    transport.heartbeat()
                except Exception as error:
                    self.events.put(("control_error", str(error)))
                finally:
                    self.heartbeat_pending = False
            threading.Thread(target=heartbeat, daemon=True).start()
        if transport is not None and not self.control_polling:
            self.control_polling = True
            def poll():
                # Do not let a slow command delay heartbeats or use a status
                # captured before a queue edit to deliver a stale instruction.
                with self.control_operation_lock:
                    value = transport.status()
                    if (not self.stop_requested and self.live_session is not None
                            and self.live_session.confirmed):
                        self.live_session.service(transport, value)
                        self.events.put(("chat_summary", (self.live_session.summary(),
                                                          self.live_session.take_notices())))
                    return value
            self._background(poll, "control_status")
        self._schedule(500, self._poll_control)

    def _drain_control(self):
        if self.closed:
            return
        while not self.events.empty():
            kind, value = self.events.get()
            if kind == "control_status":
                first_connection = not self.control_connected
                self.control_connected = True
                self.connection_badge.set("●  CONNECTED")
                self.robot_state.set(self._format_robot_status(value))
                if first_connection:
                    self.refresh_route()
                self._update_route_button()
            elif kind == "route_started":
                self.route_start_pending = False
                if value is None or self.stop_requested:
                    self.status.set("Start was cancelled; Stop takes priority.")
                    self._update_route_button()
                    continue
                self.position_confirmed.set(False)
                self.status.set("Route and Start accepted. Check physical movement; wheel values are requests.")
                self.robot_state.set(self._format_robot_status(value))
                self._update_route_button()
            elif kind == "robot_stopped":
                self.status.set("Stop was accepted by duck2.")
                self.robot_state.set(self._format_robot_status(value))
                self._update_route_button()
            elif kind == "chat_reply":
                self._append_chat("Duck2", value)
            elif kind == "chat_summary":
                self.queue_summary.set(value[0])
                for message in value[1]:
                    self._append_chat("Duck2", message)
            elif kind in ("control_error", "route_start_error"):
                # A heartbeat/status failure does not finish an in-flight
                # Start. Keep duplicate Starts blocked until that worker ends.
                if kind == "route_start_error":
                    self.route_start_pending = False
                self.control_connected = False
                self.connection_badge.set("●  CONNECTION ERROR")
                self.robot_state.set(value)
                self._update_route_button()
        self._schedule(100, self._drain_control)

    def _start_route(self, transport, plan, generation, live):
        if transport is None or not plan:
            raise RuntimeError("Connect and select a complete route first")
        with self.control_operation_lock:
            if generation != self.start_generation or self.closed:
                return None
            self.live_session = None
            self.stop_requested = False
            result = start_route(transport, plan,
                cancelled=lambda: generation != self.start_generation or self.closed,
                live_session=live)
            if result is not None and generation != self.start_generation:
                return None
            if result is not None and live is not None:
                live.observe(result)
                self.live_session = live
                self.events.put(("chat_summary", (live.summary(), live.take_notices())))
            return result

    def start_selected_route(self):
        if self.live_session is not None and self.live_session.active:
            self.status.set("End the current run with STOP DUCK2 before starting a new one.")
            return
        if self.route_start_pending:
            self.status.set("A route Start request is already in progress.")
            return
        if not (self.position_confirmed.get() and self.session.plan
                and self.session.draft_valid and self.control_connected):
            self.status.set("Connect, select both route endpoints, and confirm placement first.")
            return
        self.status.set("Sending the route to duck2…")
        self.route_start_pending = True
        self._update_route_button()
        plan, transport, generation = self.session.plan, self.transport, self.start_generation
        live = (LiveChatSession(plan.start_approach, plan.turns)
                if self.live_chat_enabled.get() else None)
        self._background(lambda: self._start_route(transport, plan, generation, live), "route_started")

    def stop_robot(self):
        if self.transport is None:
            self.status.set("No robot connection is configured; Stop was not sent.")
            return
        # Invalidates queued work immediately; the robot epoch prevents any
        # already in-flight command from overriding Stop.
        self.start_generation += 1
        self.stop_requested = True
        self.route_start_pending = False
        transport = self.transport
        def stop():
            # Send immediately, without waiting for a slow chat/route request.
            # All such requests carry the earlier epoch and cannot undo Stop.
            ack = transport.send("stop")
            with self.control_operation_lock:
                if self.live_session is not None:
                    self.live_session.cancel()
                if ack.get("accepted") is not True:
                    raise RuntimeError(str(ack.get("reason", "Stop was rejected")))
                return transport.status()
        self.status.set("Sending Stop…")
        self._background(stop, "robot_stopped")

    def _append_chat(self, speaker, message):
        self.chat_log.configure(state="normal")
        self.chat_log.insert("end", "%s: %s\n\n" % (speaker, message))
        # Keep long sessions responsive without retaining unlimited transcript.
        if int(self.chat_log.index("end-1c").split(".")[0]) > 500:
            self.chat_log.delete("1.0", "101.0")
        self.chat_log.configure(state="disabled")
        self.chat_log.see("end")

    def send_chat(self, event=None):
        text = self.message_entry.get("1.0", "end").strip()
        if not text:
            return "break"
        self.message_entry.delete("1.0", "end")
        self._append_chat("You", text)
        intent = interpret_live(text)
        if intent.action == "end":
            self.stop_robot()
            self._append_chat("Duck2", "Ending the run through Stop.")
            return "break"
        if intent.action == "clarify":
            self._append_chat("Duck2", "Please specify next left/right/straight, pause [for N seconds], "
                              "continue, slow/normal/fast speed, or quit. No command sent.")
            return "break"
        generation, transport = self.start_generation, self.transport
        def execute():
            with self.control_operation_lock:
                if generation != self.start_generation or self.closed:
                    return "Request cancelled by Stop."
                if transport is None or self.live_session is None:
                    return "Connect and Start with Live chat enabled first."
                try:
                    value = transport.poll_status()
                    reply = self.live_session.execute(intent, transport, value)
                    self.events.put(("chat_summary", (self.live_session.summary(),
                                                      self.live_session.take_notices())))
                    return reply
                except Exception as error:
                    return str(error)
        self._background(execute, "chat_reply")
        return "break"

    def start_camera(self):
        self.stop_camera()
        try:
            client = CameraClient(self.camera_url.get(), os.environ.get("DUCK2_VIEW_TOKEN", ""))
            self.poller = CameraPoller(client)
            self.poller.set_view(self.camera_view.get())
            self.poller.start()
            self.camera_state.set("Connecting to the read-only camera service…")
        except ValueError as error:
            self.camera_state.set(str(error))

    def change_camera_view(self):
        if self.poller:
            self.poller.set_view(self.camera_view.get())

    def stop_camera(self):
        if self.poller:
            self.poller.stop()
            self.poller = None
        self.photo = None
        if hasattr(self, "camera_image"):
            self.camera_image.configure(image="", text="Select Start viewing to open the camera.")
        if hasattr(self, "camera_state"):
            self.camera_state.set("Viewer stopped. No connection attempted.")

    def _fit_camera_photo(self, ppm):
        """Fill the panel while keeping the complete camera frame visible."""
        available_width = max(320, self.camera_image.winfo_width() - 8)
        available_height = max(160, self.camera_image.winfo_height() - 8)
        if Image is not None and ImageTk is not None:
            with Image.open(io.BytesIO(ppm)) as source:
                source.load()
                source = source.convert("RGB")
                panel_size = (available_width, available_height)
                backdrop = ImageOps.fit(source, panel_size,
                                        method=Image.Resampling.BILINEAR)
                backdrop = ImageEnhance.Brightness(
                    backdrop.filter(ImageFilter.GaussianBlur(14))).enhance(0.30)
                ratio = min(available_width / float(source.width),
                            available_height / float(source.height))
                size = (max(1, int(source.width * ratio)),
                        max(1, int(source.height * ratio)))
                foreground = source.resize(size, Image.Resampling.LANCZOS)
                position = ((available_width - size[0]) // 2,
                            (available_height - size[1]) // 2)
                backdrop.paste(foreground, position)
            return ImageTk.PhotoImage(backdrop)

        source = tk.PhotoImage(data=ppm, format="PPM")
        shrink = max(1, int(math.ceil(max(
            source.width() / float(available_width),
            source.height() / float(available_height)))))
        if shrink > 1:
            return source.subsample(shrink)
        grow = max(1, int(min(available_width / float(source.width()),
                              available_height / float(source.height()))))
        return source.zoom(grow) if grow > 1 else source

    def _poll_camera(self):
        if self.closed:
            return
        if self.poller:
            item = self.poller.latest()
            if item:
                kind, value = item
                if kind == "error":
                    self.camera_state.set(value)
                else:
                    try:
                        self.photo = self._fit_camera_photo(value.ppm)
                        self.camera_image.configure(image=self.photo, text="")
                        state = "STALE" if value.age > 0.5 else "Live"
                        self.camera_state.set("%s %s view • frame age %.2fs • %s" %
                                              (state, value.view, value.age,
                                               value.diagnostic or "no diagnostic"))
                    except tk.TclError:
                        self.camera_state.set("Camera frame could not be displayed")
        self._schedule(100, self._poll_camera)

    def close(self):
        self.closed = True
        self.stop_requested = True
        for identifier in self._after_calls:
            self.root.after_cancel(identifier)
        self._after_calls.clear()
        self.start_generation += 1
        self.stop_camera()
        if self.transport is not None:
            try:
                self.transport.send("stop")
            except Exception:
                # Loss of the .5-second heartbeat independently stops motion;
                # shutdown must still close even if the tunnel is already gone.
                pass
        self.session.reset()
        self.root.destroy()


def main():
    import argparse
    argparse.ArgumentParser(description="Duck2 map, camera and live-chat companion").parse_args()
    root = tk.Tk()
    CompanionWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()
