"""Exercise real Tk callbacks through the isolated onboard HTTP/ROS controller."""
import json
import os
from pathlib import Path
import sys
import time
import tkinter as tk

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "laptop"))
from duck2_companion import CompanionWindow
from chat_core import request_json


def main():
    assert request_json("http://127.0.0.1:18765/bench/scene")["bench_simulation"] is True
    root = tk.Tk()
    app = CompanionWindow(root, bench=True)
    root.geometry("1100x760+80+80")
    results = []
    def wait(predicate, label, timeout=12):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            root.update()
            if predicate():
                return
            time.sleep(.03)
        raise AssertionError(label + ": " + app.robot_state.get())
    def hold(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            root.update()
            time.sleep(.03)
    def status():
        return (app.live_session.last_status if app.live_session else {}) or {}
    def live():
        return status().get("live_session", {})
    def say(text):
        app.message_entry.insert("1.0", text)
        app.send_chat()
    def scene(name):
        request_json("http://127.0.0.1:18765/bench/scene", {"scene": name})
    def passed(label):
        results.append(label)
        print("PASS: " + label, flush=True)
    def start():
        scene("straight")
        hold(1)
        app.choose_start("A->D")
        app.choose_destination("A->D")
        app.position_confirmed.set(True)
        app.start_selected_route()
        wait(lambda: app.live_session is not None and app.live_session.active and live().get("straight"), "Start/straight")
    try:
        app.connect_control()
        wait(lambda: app.control_connected, "Connect")
        app.start_camera()
        wait(lambda: app.photo is not None, "Camera rendering")
        passed("Actual app connected and rendered synthetic camera")
        start()
        say("The next turn should be right")
        wait(lambda: app.live_session.queue == ["right"], "replace queue")
        say("Left after that one")
        wait(lambda: app.live_session.queue == ["right", "left"], "append queue")
        say("Next left")
        wait(lambda: "No left exit" in app.chat_log.get("1.0", "end"), "T-junction rejection")
        assert app.live_session.queue == ["right", "left"]
        passed("Typed queue replacement/append and invalid map exit rejection")
        say("fast speed")
        wait(lambda: live().get("profile") == "fast", "fast profile")
        say("normal speed")
        wait(lambda: live().get("profile") == "normal", "normal profile")
        scene("curve")
        wait(lambda: not live().get("straight"), "curve recognition")
        say("fast speed")
        wait(lambda: "Speed increases are available only" in app.chat_log.get("1.0", "end"), "curve speed rejection")
        say("Pause for 3 seconds")
        wait(lambda: live().get("pause_pending"), "pause pending")
        assert not live().get("paused")
        scene("straight")
        wait(lambda: live().get("paused"), "deferred pause")
        wait(lambda: not live().get("paused") and not live().get("pause_pending"), "timed resume")
        assert app.live_session.queue == ["right", "left"]
        passed("Straight-only speed selection and deferred timed pause preserve queue")
        say("stop")
        wait(lambda: live().get("paused"), "typed stop pauses")
        say("continue")
        wait(lambda: not live().get("paused"), "typed continue resumes")
        passed("Typed stop/continue use pause, not end-run")
        scene("red")
        wait(lambda: app.live_session.approach == "D->B", "reported right completion", timeout=20)
        assert app.live_session.queue == ["left"]
        passed("Red-line instruction sent once; completion advances laptop lane")
        say("quit")
        wait(lambda: not app.live_session.active, "quit")
        passed("Typed quit ends the run")
        start()
        scene("red")
        wait(lambda: status().get("state") == "red_stop", "empty red stop")
        wait(lambda: "Where should I go next?" in app.chat_log.get("1.0", "end"), "empty queue prompt")
        began = time.monotonic()
        wait(lambda: not app.live_session.active, "30-second red deadline", timeout=34)
        assert time.monotonic()-began >= 25
        passed("Empty red-line queue asks for direction and expires on real 30-second clock")
        start()
        say("pause")
        wait(lambda: live().get("paused"), "indefinite pause")
        began = time.monotonic()
        wait(lambda: not app.live_session.active, "30-second pause deadline", timeout=34)
        assert time.monotonic()-began >= 25
        passed("Indefinite pause expires after real 30-second wait")
        start()
        scene("camera_loss")
        hold(2)
        value = app.transport.status()
        assert value["wheel_speeds"] == [0., 0.] and not value["camera_valid"]
        passed("Missing synthetic camera stops wheel requests")
        scene("straight")
        app.stop_robot()
        wait(lambda: not app.live_session.active, "Stop button")
        start()
        # Freeze Tk polling deliberately: no laptop heartbeat reaches the controller.
        time.sleep(3)
        value = app.transport.status()
        assert value["wheel_speeds"] == [0., 0.] and value["manual_stop"]
        passed("App heartbeat interruption stops simulated controller")
        app.close()
        value = request_json("http://127.0.0.1:18765/status")
        assert value["wheel_speeds"] == [0., 0.]
        passed("App close delivers Stop")
    finally:
        if not app.closed:
            app.close()
        directory = Path(os.environ["LOCALAPPDATA"]) / "Duck2/bench-checks"
        path = directory / (time.strftime("%Y%m%dT%H%M%S") + "-interactive-ui.json")
        path.write_text(json.dumps({"passed": len(results) == 11, "passed_checks": results,
                                   "physical_motion": False}, indent=2), encoding="utf-8")
        print("RESULTS: " + str(path), flush=True)


if __name__ == "__main__":
    main()
