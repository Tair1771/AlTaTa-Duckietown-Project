"""Actual app map-only demo rehearsal against the isolated interactive bench."""
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
    url = "http://127.0.0.1:18765"
    assert request_json(url+"/bench/scene")["bench_simulation"] is True
    root = tk.Tk()
    app = CompanionWindow(root, bench=True)
    app.live_chat_enabled.set(False)
    results = []
    def value():
        return request_json(url+"/status")
    def wait(check, label, seconds=15):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            root.update()
            if check():
                return
            time.sleep(.05)
        raise AssertionError(label+": "+json.dumps(value()))
    def hold(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            root.update()
            time.sleep(.05)
    def scene(name):
        request_json(url+"/bench/scene", {"scene": name})
    def passed(label):
        results.append(label)
        print("PASS: "+label, flush=True)
    try:
        scene("straight")
        app.connect_control()
        app.start_camera()
        wait(lambda: app.control_connected and app.photo is not None, "app connection")
        app.choose_start("A->B")
        app.choose_destination("C->E")
        plan = app.session.plan
        assert plan is not None and len(plan.turns) >= 2
        app.position_confirmed.set(True)
        app.start_selected_route()
        wait(lambda: value()["state"] == "following" and max(value()["wheel_speeds"]) > 0, "map-only Start")
        assert app.live_session is None
        passed("Map-only Start sends complete selected route without a live-chat session")
        for index in range(1, len(plan.route)-1):
            scene("red")
            wait(lambda: value()["state"] == "red_stop", "junction dwell")
            assert value()["wheel_speeds"] == [0., 0.]
            wait(lambda: value()["route_index"] == index+1 and value()["state"] == "following",
                 "crossing and outgoing lane", seconds=20)
            hold(1)
        passed("Sequential junctions dwell, cross unmarked space and advance the route")
        scene("red")
        wait(lambda: value()["state"] == "route_complete", "destination stop")
        hold(3)
        assert value()["wheel_speeds"] == [0., 0.]
        assert value()["current_approach"] == plan.destination_approach
        passed("Destination red line completes the selected route and remains stopped")
        app.connect_control()
        hold(1)
        assert value()["state"] == "route_complete" and value()["wheel_speeds"] == [0., 0.]
        passed("Reconnect at destination does not resume movement")
        app.stop_robot()
        wait(lambda: value()["manual_stop"] is True, "manual Stop")
        assert value()["wheel_speeds"] == [0., 0.]
        passed("STOP DUCK2 remains effective after route completion")
    finally:
        app.close()
        output = Path(os.environ["LOCALAPPDATA"]) / "Duck2/bench-checks"
        output.mkdir(parents=True, exist_ok=True)
        path = output / (time.strftime("%Y%m%dT%H%M%S")+"-map-demo.json")
        path.write_text(json.dumps({"passed": len(results) == 5, "checks": results,
                                   "physical_motion": False}, indent=2))
        print("RESULTS: "+str(path), flush=True)


if __name__ == "__main__":
    main()
