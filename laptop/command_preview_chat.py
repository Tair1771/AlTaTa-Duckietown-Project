"""Separate Windows chat window for offline command previews."""
import json
import tkinter as tk
from tkinter import ttk
from command_preview import PreviewSession


class CommandPreviewWindow:
    def __init__(self, root):
        self.root = root
        self.session = PreviewSession()
        root.title("Offline command preview — no robot connection")
        root.geometry("880x800")
        root.minsize(700, 650)
        panel = ttk.Frame(root, padding=16)
        panel.pack(fill="both", expand=True)
        ttk.Label(panel, text="Offline command preview — no robot connection",
                  font=("Segoe UI", 17, "bold"), wraplength=800).pack(anchor="w")
        ttk.Label(panel, text="No movement • No API key • Records are local tests, not controller acceptance",
                  wraplength=800).pack(anchor="w", pady=8)
        self.log = tk.Text(panel, height=12, wrap="word", state="disabled")
        self.log.pack(fill="both", expand=True)
        composer = ttk.Frame(panel)
        composer.pack(fill="x", pady=8)
        self.message = tk.StringVar()
        entry = ttk.Entry(composer, textvariable=self.message)
        entry.pack(side="left", fill="x", expand=True)
        entry.bind("<Return>", self.send)
        ttk.Button(composer, text="Preview", command=self.send).pack(side="right", padx=5)
        self.details = self.text_panel(panel, "Current draft — no delivery", 9)
        self.record_button = ttk.Button(panel, text="Record preview", command=self.record, state="disabled")
        self.record_button.pack(anchor="e", pady=5)
        self.history = self.text_panel(panel, "Local preview records — no simulated movement", 7)
        ttk.Button(panel, text="Clear conversation and records", command=self.clear).pack(anchor="e", pady=8)
        self.append("duck2", "Try 'slow down', 'take the next right', or 'stop'. Chatting only prepares a draft. Record preview saves a local test entry. Nothing is sent or executed.")
        root.protocol("WM_DELETE_WINDOW", self.close)
        entry.focus_set()

    def text_panel(self, parent, title, height):
        group = ttk.LabelFrame(parent, text=title, padding=5)
        group.pack(fill="x")
        text = tk.Text(group, height=height, wrap="word", state="disabled")
        text.pack(fill="x")
        return text

    @staticmethod
    def replace(widget, text):
        widget.configure(state="normal")
        widget.delete("1.0", "end")
        widget.insert("end", text)
        widget.configure(state="disabled")

    def append(self, who, text):
        self.log.configure(state="normal")
        self.log.insert("end", who + ": " + text + "\n\n")
        self.log.configure(state="disabled")
        self.log.see("end")

    def refresh(self, latest=None):
        draft = self.session.draft
        detail = draft or latest
        self.replace(self.details, json.dumps(detail.to_dict(), indent=2) if detail else "No draft.")
        self.record_button.configure(state="normal" if draft else "disabled")
        rows = ["Record %s: %s %s — local only; acceptance not requested; completion unknown" %
                (r["record_number"], r["preview"]["action"], json.dumps(r["preview"]["parameters"]))
                for r in self.session.receiver.records]
        self.replace(self.history, "\n".join(rows))

    def send(self, event=None):
        text = self.message.get().strip()
        if text:
            self.message.set("")
            self.append("You", text)
            result = self.session.chat(text)
            self.append("duck2", result.explanation + " Offline preview only; nothing was sent or executed.")
            self.refresh(result)
        return "break"

    def record(self):
        try:
            self.session.record()
            self.append("duck2", "Preview recorded locally. Controller acceptance was not requested; physical completion is unknown.")
        except ValueError as error:
            self.append("duck2", str(error))
        self.refresh()

    def clear(self):
        self.session.clear()
        self.replace(self.log, "")
        self.message.set("")
        self.refresh()

    def close(self):
        self.session.clear()
        self.root.destroy()


def main():
    root = tk.Tk()
    CommandPreviewWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()
