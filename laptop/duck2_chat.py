#!/usr/bin/env python3
"""Native Windows chat and direct controls. No third-party Python packages required."""
import os
import queue
import threading
import tkinter as tk
from tkinter import ttk
from chat_core import ChatSession, OpenAIInterpreter, RobotTransport

class ChatWindow:
    def __init__(self, root):
        self.root = root
        root.title("duck2 · Driving companion")
        root.geometry("850x740")
        root.minsize(730,650)
        self.events = queue.Queue()
        self.session = None
        self.polling = False
        self.connected = False
        self.closed = False
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure("TButton", padding=8)
        style.configure("TLabel", font=("Segoe UI", 10))
        panel = ttk.Frame(root, padding=20)
        panel.pack(fill="both", expand=True)
        ttk.Label(panel, text="duck2 / Driving companion",
                  font=("Segoe UI", 22, "bold")).pack(anchor="w")
        ttk.Label(panel, text="Talk about speed and the next turn. Follow the bot's live response.").pack(anchor="w",pady=(5,15))
        settings = ttk.LabelFrame(panel, text="Connection", padding=10)
        settings.pack(fill="x")
        self.url = tk.StringVar(value="http://127.0.0.1:8765")
        self.token = tk.StringVar(value=os.environ.get("DUCK2_CONTROL_TOKEN",""))
        self.key = tk.StringVar(value=os.environ.get("OPENAI_API_KEY",""))
        for row,(label,variable,hidden) in enumerate([
                ("Robot gateway",self.url,False), ("Control token",self.token,True),
                ("OpenAI API key",self.key,True)]):
            ttk.Label(settings,text=label).grid(row=row,column=0,sticky="w",padx=(0,10))
            ttk.Entry(settings,textvariable=variable,show="*" if hidden else "").grid(
                row=row,column=1,sticky="ew",pady=3)
        settings.columnconfigure(1,weight=1)
        ttk.Button(settings,text="Connect",command=self.connect).grid(row=0,column=2,rowspan=3,padx=10)
        self.state = tk.StringVar(value="Disconnected — no robot commands sent.")
        ttk.Label(panel,textvariable=self.state,wraplength=790).pack(fill="x",pady=12)
        controls = ttk.Frame(panel)
        controls.pack(fill="x")
        tk.Button(controls,text="STOP",bg="#bf3434",fg="white",font=("Segoe UI",12,"bold"),
                  padx=22,pady=8,command=self.stop).pack(side="left",padx=(0,10))
        for label,action in [("Continue","continue"),("Slower","slow_down"),("Faster","speed_up")]:
            ttk.Button(controls,text=label,command=lambda a=action:self.direct(a)).pack(side="left",padx=4)
        ttk.Button(controls,text="View route map",command=self.show_map).pack(side="right")
        route = ttk.LabelFrame(panel,text="Starting position",padding=10)
        route.pack(fill="x",pady=12)
        self.route = tk.StringVar(value="A D C E A")
        ttk.Entry(route,textvariable=self.route,width=28).pack(side="left",padx=(0,8))
        self.placed = tk.BooleanVar(value=False)
        ttk.Checkbutton(route,text="Placed after the first junction, toward the second",
                        variable=self.placed).pack(side="left")
        ttk.Button(route,text="Set route",command=self.set_route).pack(side="right")
        self.log = tk.Text(panel,wrap="word",height=12,font=("Segoe UI",11),state="disabled",
                           bg="#f7f8f5",relief="flat",padx=12,pady=12)
        self.log.pack(fill="both",expand=True)
        self.append("duck2", "Connect to the gateway to see live state. Chat needs an API key; direct controls do not. Junction movement also requires calibration.")
        composer=ttk.Frame(panel)
        composer.pack(fill="x",pady=(12,0))
        self.message=tk.StringVar()
        entry=ttk.Entry(composer,textvariable=self.message,font=("Segoe UI",12))
        entry.pack(side="left",fill="x",expand=True,padx=(0,8))
        entry.bind("<Return>",lambda e:self.send())
        ttk.Button(composer,text="Send",command=self.send).pack(side="right")
        root.protocol("WM_DELETE_WINDOW",self.close)
        root.after(100,self.drain)
        root.after(500,self.poll)

    def show_map(self):
        window=tk.Toplevel(self.root)
        window.title("duck2 · Route map")
        ttk.Label(window,text="Proposed route · position is not automatically localized",
                  padding=12).pack()
        canvas=tk.Canvas(window,width=480,height=530,bg="#19211f",highlightthickness=0)
        canvas.pack(padx=12,pady=(0,12))
        nodes={"A":(65,300),"B":(250,300),"C":(410,300),"D":(410,175),"E":(250,455)}
        roads={
            ("A","B"):[65,300,250,300],
            ("B","C"):[250,300,410,300],
            ("B","D"):[250,300,250,215,285,175,410,175],
            ("C","D"):[410,300,410,175],
            ("B","E"):[250,300,250,455],
            ("A","E"):[65,300,65,420,95,455,250,455],
            ("C","E"):[410,300,410,420,380,455,250,455],
            ("A","D"):[65,300,65,220,95,195,130,195,150,170,
                       150,130,180,105,180,70,215,45,370,45,410,80,410,175],
        }
        for points in roads.values():
            canvas.create_line(*points,fill="#475550",width=24,joinstyle="round")
            canvas.create_line(*points,fill="#dfc96c",width=2,dash=(6,6))
        route=self.route.get().upper().replace(","," ").split()
        for a,b in zip(route,route[1:]):
            points=roads.get((a,b))
            if points is None and (b,a) in roads:
                pairs=list(zip(roads[(b,a)][::2],roads[(b,a)][1::2]))
                points=[coordinate for pair in reversed(pairs) for coordinate in pair]
            if points:
                canvas.create_line(*points,fill="#71d7a6",width=5,arrow="last",
                                   arrowshape=(12,15,6),joinstyle="round")
        for name,(x,y) in nodes.items():
            canvas.create_oval(x-15,y-15,x+15,y+15,fill="#f8faf5",outline="")
            canvas.create_text(x,y,text=name,fill="#16241c",font=("Segoe UI",13,"bold"))
        canvas.create_text(240,505,text=" → ".join(route),fill="#effaf3",
                           font=("Segoe UI",14,"bold"))
        ttk.Label(window,text="A: left middle    B: center    C: right middle\n"
                  "D: upper right    E: bottom center\n"
                  "Simplified road connections from your map; not a scaled physical pose.",
                  justify="center",padding=(12,0,12,12)).pack()

    def append(self,who,text):
        self.log.configure(state="normal")
        self.log.insert("end",who+": "+text+chr(10)*2)
        self.log.see("end")
        self.log.configure(state="disabled")

    def background(self,operation,kind="reply"):
        def work():
            try:
                self.events.put((kind,operation()))
            except Exception as error:
                self.events.put(("error",str(error)))
            finally:
                if kind=="status":
                    self.polling=False
        threading.Thread(target=work,daemon=True).start()

    def connect(self):
        if self.session:
            self.session._generation += 1
        self.session=ChatSession(RobotTransport(self.url.get(),self.token.get()),
                                 OpenAIInterpreter(self.key.get()))
        self.background(self.session.transport.poll_status,"status")

    def stop(self):
        if self.session:
            self.background(self.session.stop)
        else:
            self.append("duck2","No connection configured; stop could not be sent.")

    def direct(self,action):
        if not self.session:
            self.append("duck2","Connect first.")
            return
        session=self.session
        def command():
            status=session.transport.status()
            return session.describe_ack(session.transport.send(
                action,expected_control_epoch=status["control_epoch"]))
        self.background(command)

    def set_route(self):
        if not self.session or not self.placed.get():
            self.append("duck2","Confirm the physical starting position before setting the route.")
            return
        route=self.route.get().upper().replace(","," ").split()
        session=self.session
        self.background(lambda:session.describe_ack(session.transport.send(
            "set_route",route=route,position_confirmed=True)))
        self.placed.set(False)

    def send(self):
        text=self.message.get().strip()
        if not text:
            return
        if not self.session:
            self.append("duck2","Connect first.")
            return
        self.message.set("")
        self.append("You",text)
        session=self.session
        self.background(lambda:session.chat(text))

    def poll(self):
        if self.closed:
            return
        if self.session and not self.polling:
            self.polling=True
            self.background(self.session.transport.poll_status,"status")
        self.root.after(500,self.poll)

    def drain(self):
        if self.closed:
            return
        while not self.events.empty():
            kind,value=self.events.get()
            if kind=="status":
                self.connected=True
                self.state.set("State: %s  |  Next: %s  |  Speed: %.0f%%  |  Wheels: %s\nRoute: %s%s" % (
                    value["state"],value["next_junction"] or "—",100*value["speed_scale"],
                    value["wheel_speeds"]," → ".join(value["route"]),
                    ("  ·  LAPTOP CONNECTION LOST" if value.get("client_connection_lost") else
                     "  ·  OBSTACLE STOP" if value.get("obstacle_stop") else
                     "  ·  MANUALLY STOPPED" if value["manual_stop"] else "")))
            elif kind=="error":
                if self.connected:
                    self.connected=False
                # Keep errors visible without flooding the transcript on every status poll.
                self.state.set(value)
            else:
                self.append("duck2",value)
        self.root.after(100,self.drain)

    def close(self):
        self.closed=True
        if not self.session:
            self.root.destroy()
            return
        self.session._generation += 1
        self.state.set("Sending stop before closing...")
        done=threading.Event()
        outcome=[]
        def finish():
            try:
                outcome.append(self.session.stop())
            except Exception as error:
                outcome.append("Stop was not acknowledged: "+str(error))
            done.set()
        threading.Thread(target=finish,daemon=False).start()
        def check():
            if done.is_set():
                if "not acknowledged" in outcome[0] or outcome[0].startswith("Not applied"):
                    from tkinter import messagebox
                    messagebox.showerror("Stop not confirmed",outcome[0])
                self.root.destroy()
            else:
                self.root.after(100,check)
        check()

if __name__=="__main__":
    root=tk.Tk()
    ChatWindow(root)
    root.mainloop()
