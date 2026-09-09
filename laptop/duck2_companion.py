#!/usr/bin/env python3
"""Offline-first Windows companion: route map, camera viewer and local chat."""

import io
import math
import os
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
from companion_core import OfflineCompanionSession
from route_planner import (MAP_NODES, MAP_ROADS, RED_LINE_APPROACHES,
                           approach_id, parse_approach)


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
        self.poller = None
        self.photo = None
        self.closed = False
        self._workspace_split_initialized = False
        root.title("duck2 companion")
        root.geometry("1100x760")
        root.minsize(850, 600)
        root.configure(background=COLORS["background"])
        self._configure_style()

        outer = ttk.Frame(root, padding=(18, 16, 18, 16), style="App.TFrame")
        outer.pack(fill="both", expand=True)

        header = ttk.Frame(outer, padding=(18, 14), style="Header.TFrame")
        header.pack(fill="x", pady=(0, 10))
        heading = ttk.Frame(header, style="Header.TFrame")
        heading.pack(side="left", fill="x", expand=True)
        ttk.Label(heading, text="duck2 companion", style="HeaderTitle.TLabel").pack(anchor="w")
        ttk.Label(heading, text="Plan a route, inspect the camera, and talk through commands.",
                  style="HeaderSubtitle.TLabel").pack(anchor="w", pady=(2, 0))
        ttk.Label(header, text="●  OFFLINE & READ ONLY",
                  style="Safety.TLabel").pack(side="right", padx=(12, 0))

        self.status = tk.StringVar(value="Select the directed starting lane and a red-line destination.")
        status_bar = ttk.Frame(outer, padding=(13, 9), style="Status.TFrame")
        status_bar.pack(fill="x", pady=(0, 10))
        ttk.Label(status_bar, text="STATUS", style="StatusCaption.TLabel").pack(side="left")
        ttk.Label(status_bar, textvariable=self.status, style="StatusText.TLabel",
                  wraplength=900).pack(side="left", padx=(12, 0), fill="x", expand=True)

        self.notebook = ttk.Notebook(outer, style="App.TNotebook")
        self.notebook.pack(fill="both", expand=True)
        self.map_tab = ttk.Frame(self.notebook, padding=12, style="App.TFrame")
        self.workspace_tab = ttk.Frame(self.notebook, padding=10, style="App.TFrame")
        self.notebook.add(self.map_tab, text="  1  Map & route  ")
        self.notebook.add(self.workspace_tab, text="  2  Camera & chat  ")
        self.notebook.bind("<<NotebookTabChanged>>", self._notebook_tab_changed)
        self.workspace_panes = ttk.Panedwindow(self.workspace_tab, orient="horizontal")
        self.workspace_panes.pack(fill="both", expand=True)
        self.camera_tab = ttk.Frame(self.workspace_panes, padding=(8, 6), style="App.TFrame")
        self.chat_tab = ttk.Frame(self.workspace_panes, padding=(8, 6), style="App.TFrame")
        self.workspace_panes.add(self.camera_tab, weight=7)
        self.workspace_panes.add(self.chat_tab, weight=1)
        self._build_map()
        self._build_camera()
        self._build_chat()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(100, self._poll_camera)
        root.after_idle(self._set_workspace_split)

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
            self.workspace_panes.sashpos(0, int(width * 0.72))
            self._workspace_split_initialized = True

    def _notebook_tab_changed(self, event=None):
        """Set the initial split only after the shared workspace is visible."""
        if self.notebook.select() == str(self.workspace_tab):
            self.root.after_idle(self._set_workspace_split)

    def _build_map(self):
        ttk.Label(self.map_tab, text="Choose where duck2 starts and finishes",
                  style="Section.TLabel").pack(anchor="w")
        ttk.Label(self.map_tab,
                  text="Select a right-lane direction first, then select a red-line destination.",
                  style="Hint.TLabel").pack(anchor="w", pady=(2, 10))

        controls = ttk.Frame(self.map_tab, style="App.TFrame")
        controls.pack(fill="x", pady=(0, 10))
        self.selection_mode = tk.StringVar(value="start")
        values = list(RED_LINE_APPROACHES)
        self.start_value = tk.StringVar()
        self.destination_value = tk.StringVar()

        start_card = ttk.LabelFrame(controls, text="1  Starting lane", padding=(12, 8),
                                    style="Card.TLabelframe")
        start_card.pack(side="left", fill="x", expand=True, padx=(0, 6))
        ttk.Radiobutton(start_card, text="Click an arrow on the map", value="start",
                        variable=self.selection_mode).pack(side="left")
        start = ttk.Combobox(start_card, textvariable=self.start_value, values=values,
                             state="readonly", width=10)
        start.pack(side="right", padx=(8, 0))

        destination_card = ttk.LabelFrame(controls, text="2  Destination", padding=(12, 8),
                                          style="Card.TLabelframe")
        destination_card.pack(side="left", fill="x", expand=True, padx=6)
        ttk.Radiobutton(destination_card, text="Click a red marker", value="destination",
                        variable=self.selection_mode).pack(side="left")
        destination = ttk.Combobox(destination_card, textvariable=self.destination_value,
                                   values=values, state="readonly", width=10)
        destination.pack(side="right", padx=(8, 0))
        ttk.Button(controls, text="Clear", command=self.clear_route,
                   style="Secondary.TButton").pack(side="right", padx=(6, 0))
        start.bind("<<ComboboxSelected>>", lambda event: self.choose_start(self.start_value.get()))
        destination.bind("<<ComboboxSelected>>",
                         lambda event: self.choose_destination(self.destination_value.get()))

        body = ttk.Frame(self.map_tab, style="App.TFrame")
        body.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(body, width=550, height=520, bg=COLORS["map"],
                                highlightthickness=1, highlightbackground=COLORS["border"])
        self.canvas.pack(side="left", padx=(0, 12))
        self.canvas.bind("<Button-1>", self.map_click)
        side = ttk.LabelFrame(body, text="Route preview", padding=14,
                              style="Card.TLabelframe")
        side.pack(side="left", fill="both", expand=True)
        self.route_text = tk.Text(side, width=36, height=14, wrap="word", state="disabled",
                                  relief="flat", borderwidth=0, background=COLORS["card"],
                                  foreground=COLORS["ink"], font=("Segoe UI", 11),
                                  padx=4, pady=4)
        self.route_text.pack(fill="both", expand=True, pady=6)
        self.start_route_button = ttk.Button(
            side, text="Start route  •  locked until track validation",
            state="disabled", style="Primary.TButton")
        self.start_route_button.pack(fill="x", pady=(6, 8))
        ttk.Label(side, text=("A route here is a local draft. Shortest means fewest junctions; "
                              "the app does not know duck2's live position."),
                  style="CardHint.TLabel", wraplength=360).pack(anchor="w")
        self.draw_map()
        self.refresh_route()

    def _build_camera(self):
        camera_heading = ttk.Frame(self.camera_tab, style="App.TFrame")
        camera_heading.pack(fill="x", pady=(0, 6))
        ttk.Label(camera_heading, text="Live camera",
                  style="Section.TLabel").pack(side="left")
        ttk.Label(camera_heading, text="Read-only SSH connection",
                  style="Hint.TLabel").pack(side="right", padx=(8, 0))

        top = ttk.LabelFrame(self.camera_tab, text="Camera connection", padding=(10, 7),
                             style="Card.TLabelframe")
        top.pack(fill="x", pady=(0, 7))
        connection_row = ttk.Frame(top, style="Card.TFrame")
        connection_row.pack(fill="x")
        self.camera_url = tk.StringVar(value="http://127.0.0.1:8766")
        ttk.Label(connection_row, text="Local address").pack(side="left")
        ttk.Entry(connection_row, textvariable=self.camera_url, width=28).pack(
            side="left", padx=8, fill="x", expand=True)
        self.start_camera_button = ttk.Button(connection_row, text="Start viewing",
                                              command=self.start_camera,
                                              style="Primary.TButton")
        self.start_camera_button.pack(side="left")
        ttk.Button(connection_row, text="Stop", command=self.stop_camera,
                   style="Secondary.TButton").pack(side="left", padx=(6, 12))
        ttk.Label(connection_row, text="View").pack(side="left", padx=(0, 3))
        self.camera_view = tk.StringVar(value="normal")
        for view in CameraClient.VIEWS:
            ttk.Radiobutton(connection_row, text=view.title(), value=view,
                            variable=self.camera_view,
                            command=self.change_camera_view).pack(side="left", padx=2)
        self.camera_state = tk.StringVar(value="Viewer stopped. No connection attempted.")
        camera_status = ttk.Frame(self.camera_tab, padding=(10, 6), style="Status.TFrame")
        camera_status.pack(fill="x", pady=(0, 7))
        ttk.Label(camera_status, text="CAMERA", style="StatusCaption.TLabel").pack(side="left")
        ttk.Label(camera_status, textvariable=self.camera_state, style="StatusText.TLabel",
                  wraplength=850).pack(side="left", padx=(12, 0), fill="x", expand=True)
        self.camera_image = ttk.Label(self.camera_tab, anchor="center", background="#16231f")
        self.camera_image.pack(fill="both", expand=True)

    def _build_chat(self):
        self.chat_tab.columnconfigure(0, weight=1)
        self.chat_tab.rowconfigure(1, weight=1)
        chat_header = ttk.Frame(self.chat_tab, style="App.TFrame")
        chat_header.grid(row=0, column=0, sticky="ew", pady=(0, 5))
        ttk.Label(chat_header, text="Chat", style="Section.TLabel").pack(anchor="w")
        ttk.Label(chat_header, text="Offline draft • nothing is sent to duck2",
                  style="Hint.TLabel").pack(anchor="w")
        self.chat_log = tk.Text(self.chat_tab, width=30, wrap="word", state="disabled",
                                font=("Segoe UI", 11), padx=16, pady=14, relief="flat",
                                borderwidth=1, background=COLORS["card"],
                                foreground=COLORS["ink"])
        self.chat_log.grid(row=1, column=0, sticky="nsew")
        self.chat_log.tag_configure("name_you", foreground=COLORS["green"],
                                    font=("Segoe UI", 10, "bold"), spacing1=8)
        self.chat_log.tag_configure("name_duck2", foreground="#7d5d17",
                                    font=("Segoe UI", 10, "bold"), spacing1=8)
        self.chat_log.tag_configure("message", lmargin1=12, lmargin2=12,
                                    spacing3=8)
        self.append_chat("duck2", "I can plan between red lines, explain commands, and revise the next turn. Everything here is an offline draft.")
        composer = ttk.Frame(self.chat_tab, style="App.TFrame")
        composer.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        composer.columnconfigure(0, weight=1)
        composer.columnconfigure(1, weight=1)
        self.message = tk.StringVar()
        self.message_entry = ttk.Entry(composer, textvariable=self.message,
                                       style="ChatInput.TEntry", font=("Segoe UI", 11))
        self.message_entry.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 6))
        self.message_entry.bind("<Return>", self.send_chat)
        ttk.Button(composer, text="Interpret message", command=self.send_chat,
                   style="Primary.TButton").grid(row=1, column=0, sticky="ew", padx=(0, 3))
        ttk.Button(composer, text="Clear", command=self.clear_all,
                   style="Secondary.TButton").grid(row=1, column=1, sticky="ew", padx=(3, 0))

    def _replace(self, widget, text):
        widget.configure(state="normal")
        widget.delete("1.0", "end")
        widget.insert("end", text)
        widget.configure(state="disabled")

    def append_chat(self, who, text):
        self.chat_log.configure(state="normal")
        tag = "name_you" if who.lower() == "you" else "name_duck2"
        self.chat_log.insert("end", who.upper() + "\n", tag)
        self.chat_log.insert("end", text + "\n", "message")
        self.chat_log.see("end")
        self.chat_log.configure(state="disabled")

    def choose_start(self, approach):
        try:
            reply = self.session.select_start(approach)
            self.start_value.set(approach)
            self.status.set(reply.explanation)
        except ValueError as error:
            self.status.set(str(error))
        self.refresh_route()

    def choose_destination(self, approach):
        try:
            reply = self.session.select_destination(approach)
            self.destination_value.set(approach)
            self.status.set(reply.explanation)
        except ValueError as error:
            self.status.set(str(error))
        self.refresh_route()

    def map_click(self, event):
        if self.selection_mode.get() == "destination":
            candidates = [(math.hypot(event.x - red_line_point(value)[0],
                                      event.y - red_line_point(value)[1]), value)
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
            distance = min(point_segment_distance(event.x, event.y, *first, *second)
                           for first, second in zip(points, points[1:]))
            candidates.append((distance, value))
        distance, value = min(candidates)
        if distance <= 15:
            self.choose_start(value)
        else:
            self.status.set("Click one of the arrowed right-lane paths to select the start direction.")

    def draw_map(self):
        self.canvas.delete("all")
        for x in range(25, 551, 50):
            self.canvas.create_line(x, 0, x, 520, fill="#1c2c27", width=1)
        for y in range(25, 521, 50):
            self.canvas.create_line(0, y, 550, y, fill="#1c2c27", width=1)
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
        self.canvas.create_text(275, 440,
            text="Blue = start  •  Green = route  •  Red = destination",
            fill="#eef5f0", font=("Segoe UI", 9, "bold"))

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
            text = ("Start lane: %s\nDestination red line: %s\n\nPath: %s\n"
                    "Junctions crossed: %d\n\n%s\n\nOFFLINE DRAFT — no command sent"
                    % (plan.start_approach, plan.destination_approach,
                       " → ".join(plan.route), plan.junction_count,
                       "\n".join(steps) if steps else "Already on the destination approach."))
        self._replace(self.route_text, text)

    def clear_route(self):
        self.session.destination_approach = self.session.plan = None
        self.session.draft_valid = False
        self.destination_value.set("")
        self.status.set("Route draft cleared. No robot movement was cancelled.")
        self.refresh_route()

    def send_chat(self, event=None):
        text = self.message.get().strip()
        if not text:
            return "break"
        self.message.set("")
        self.append_chat("You", text)
        reply = self.session.chat(text)
        self.append_chat("duck2", reply.explanation + " Interpretation only; nothing was sent.")
        if self.session.start_approach:
            self.start_value.set(self.session.start_approach)
        self.destination_value.set(self.session.destination_approach or "")
        self.status.set(reply.explanation)
        self.refresh_route()
        return "break"

    def clear_all(self):
        self.session.reset()
        self.start_value.set("")
        self.destination_value.set("")
        self._replace(self.chat_log, "")
        self.append_chat("duck2", "Conversation and route draft cleared.")
        self.status.set("Select the directed starting lane and a red-line destination.")
        self.refresh_route()

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
            self.camera_image.configure(image="")
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
                        self.camera_image.configure(image=self.photo)
                        state = "STALE" if value.age > 0.5 else "Live"
                        self.camera_state.set("%s %s view • frame age %.2fs • %s" %
                                              (state, value.view, value.age,
                                               value.diagnostic or "no diagnostic"))
                    except tk.TclError:
                        self.camera_state.set("Camera frame could not be displayed")
        self.root.after(100, self._poll_camera)

    def close(self):
        self.closed = True
        self.stop_camera()
        self.session.reset()
        self.root.destroy()


def main():
    root = tk.Tk()
    CompanionWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()
