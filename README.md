# AlTaTa — Duckietown Final

**Tair Kezdekbayev · Alvaro Mendez Li · Tan Kayra Erol**

A Windows companion with a directed map, A* route planning, live camera and a
local chatbot, connected to duck2's onboard lane and junction controller.

- [Demo video](https://youtu.be/jS0tNWo3xnc), also in [Video Demo Link.txt](Video%20Demo%20Link.txt).
- [Final report: PDF and LaTeX](report/README.md).
- [Setup](docs/STARTUP.md), [app operation](docs/USAGE.md) and [chat commands](docs/LIVE_CHAT.md).

## Run the project

Use Windows PowerShell in this repository's root. Install Windows Python 3.10+
with Tcl/Tk and the Python launcher, then install the desktop dependency:

```powershell
py -3 -m pip install -r requirements-desktop.txt
```

1. Enable the configured hotspot, power on duck2 and let it finish booting.
2. Run `tools/Start-Duck2-Check.cmd`. For a new laptop, complete the one-time
   trusted SSH setup in [STARTUP](docs/STARTUP.md).
3. While duck2 is stationary, run `tools/Start-Duck2-DrivingMode.cmd`.
   This prepares the controller and camera connection without starting movement.
4. Open `laptop/Start-Duck2Companion.cmd`. Click **Connect to duck2** and
   **Start viewing** in Camera & status.
5. Select the actual directed starting lane and destination red line on the map.
   Confirm placement and click **Start selected route** while watching duck2.
6. Type messages in **Live chat**. **STOP DUCK2** ends the run.

The normal companion is the sole desktop launcher. Use the same checkout for
preparation and the app. Source changes require a newly prepared robot container
and a reopened app; existing processes do not reload source automatically.

Live chat is enabled by default. It asks for a direction when its turn queue
ends at a red line and waits up to 60 seconds. Uncheck Live chat before Start
for map-only operation ending at the selected destination.

## Included files

| Folder | Purpose |
| --- | --- |
| `laptop/` | Companion, map planner, camera viewer, local chat and transport |
| `packages/duckie_lane_follower/` | ROS perception, navigation, gateways and watchdog |
| `launchers/` | Continuous operation, read-only camera and idle default |
| `tools/` | SSH setup, connection, stationary preparation and optional image build |
| `config/` | Robot interfaces and pinned runtime image identities |
| `docs/` | Setup, operation and command reference |
| `report/` | Final PDF and editable LaTeX source |

No model API key or downloaded language model is required. Private keys and
connection caches belong outside this repository. Keep `LICENSE.pdf` with the
distribution. Component maneuvers have been demonstrated; complete-route
reliability remains limited by perception and physical conditions, as described
in the report.
