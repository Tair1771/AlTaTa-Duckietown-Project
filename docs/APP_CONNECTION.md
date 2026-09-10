# Stationary companion connection

For driving preparation, use `tools/Start-Duck2-DrivingMode.cmd` instead; see
[the usage guide](USAGE.md#start-a-continuous-session). It prepares a stopped
controller and suspends the preview so their ROS names and ports do not clash.

From Windows, run `tools\Start-Duck2-AppConnection.cmd`, then open
`laptop\Start-Duck2Companion.cmd`. Click **Connect to duck2** at
`http://127.0.0.1:8765` and **Start viewing** at `http://127.0.0.1:8766`.

The connection launcher uses the existing Windows SSH alias `duck2` and the
installed ARM64 ROS base. It copies our Python application into a temporary
directory and starts the separate `duck2-companion-preview` container.
It does not require Docker Desktop, `dts devel build`, or a Git operation.
Existing healthy preview services and tunnels are reused.

This is a stationary connection mode: driving is disabled and wheel output is
remapped to `/duck2/lane_follower/diagnostic_wheels_cmd`. Normal car-interface
is left in its existing state (including stopped after tests). Starting a route is intentionally blocked by the app's
wheel-ownership check. This does not establish readiness for continuous driving.

`channel ... Connection refused` means the SSH tunnel cannot reach the remote
gateway; a tunnel alone does not start that gateway. The connection launcher
checks the service before declaring success.

To end the preview, close the app and run `ssh duck2 docker stop
duck2-companion-preview`. To remove the stopped temporary container, run
`ssh duck2 docker rm duck2-companion-preview`. Before starting a driving
controller, end this preview so its ROS node names and ports do not conflict.
The container has no restart policy and does not start automatically at boot.
If it exists but is stopped, the launcher removes only that stopped temporary
container and stages current source. A running driving container blocks preview
startup. Both command and camera tunnel endpoints must be reachable. No installed
robot services are replaced. See [STARTUP](STARTUP.md) for reboot recovery.
