"""Standalone Tk chat for interpretation only. No connection or movement controls."""
import json
import tkinter as tk
from tkinter import ttk

from offline_interpreter import OfflineInterpreter


class OfflineChatWindow:
    def __init__(self, root):
        self.root = root
        self.interpreter = OfflineInterpreter()
        root.title("duck2 · Offline request interpreter")
        root.geometry("850x760")
        root.minsize(650, 600)
        panel = ttk.Frame(root, padding=20)
        panel.pack(fill="both", expand=True)
        ttk.Label(panel, text="duck2 / Understand a request", font=("Segoe UI", 21, "bold")).pack(anchor="w")
        ttk.Label(panel, text="INTERPRETATION ONLY · No robot connection · No movement · No API key",
                  wraplength=780).pack(anchor="w", pady=(8, 14))
        self.log = tk.Text(panel, height=15, wrap="word", state="disabled",
                           font=("Segoe UI", 11), padx=12, pady=12)
        self.log.pack(fill="both", expand=True)
        examples = ttk.Frame(panel)
        examples.pack(fill="x", pady=8)
        for example in ("Slow down a little", "Take the next right", "Stop after two seconds"):
            ttk.Button(examples, text=example, command=lambda text=example: self.message.set(text)).pack(side="left", padx=(0, 5))
        composer = ttk.Frame(panel)
        composer.pack(fill="x")
        self.message = tk.StringVar()
        self.entry = ttk.Entry(composer, textvariable=self.message, font=("Segoe UI", 12))
        self.entry.pack(side="left", fill="x", expand=True, padx=(0, 8))
        self.entry.bind("<Return>", self.send)
        ttk.Button(composer, text="Interpret", command=self.send).pack(side="right")
        details = ttk.LabelFrame(panel, text="Latest interpretation — description only", padding=8)
        details.pack(fill="x", pady=(14, 8))
        self.details = tk.Text(details, height=9, wrap="word", state="disabled", font=("Consolas", 10))
        self.details.pack(fill="x")
        ttk.Button(panel, text="Clear conversation", command=self.clear).pack(anchor="e")
        self.append("duck2", "Hello! I can interpret speed, turns, stops and reverse requests. Try a message, then a correction such as 'make that three seconds'. Type 'help' for ideas. Nothing here controls the robot.")
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.entry.focus_set()

    def append(self, who, text):
        self.log.configure(state="normal")
        self.log.insert("end", who + ": " + text + "\n\n")
        self.log.configure(state="disabled")
        self.log.see("end")

    def send(self, event=None):
        text = self.message.get().strip()
        if not text:
            return "break"
        self.message.set("")
        self.append("You", text)
        result = self.interpreter.interpret(text)
        self.append("duck2", result.reply)
        self.details.configure(state="normal")
        self.details.delete("1.0", "end")
        self.details.insert("end", json.dumps(result.to_dict(), indent=2, ensure_ascii=False))
        self.details.configure(state="disabled")
        return "break"

    def clear(self):
        self.interpreter.reset()
        for widget in (self.log, self.details):
            widget.configure(state="normal")
            widget.delete("1.0", "end")
            widget.configure(state="disabled")
        self.append("duck2", "Conversation cleared. Interpretation only; nothing was sent or executed.")

    def close(self):
        self.root.destroy()


def main():
    root = tk.Tk()
    OfflineChatWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()
