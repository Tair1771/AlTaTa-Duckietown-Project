# Setup and operation tools

Run from Windows PowerShell in the repository root. See [STARTUP](../docs/STARTUP.md).

| Entry | Purpose |
| --- | --- |
| `setup_duck2_ssh.ps1` | One-time protected key and strict SSH alias |
| `Start-Duck2-Check.cmd` | Read-only SSH, ROS and ownership checks |
| `Start-Duck2-DrivingMode.cmd` | Prepare stopped controller, camera and tunnel |
| `Start-Duck2-AppConnection.cmd` | Stationary camera-only connection |
| `build_local.py` | Optional local pinned Docker image build |

The shortcuts use `inspect_robot_setup.py`, `prepare_companion_driving.py` and
`connect_companion.py`. Driving preparation also requires `arm_driver_stopped.py`:
it verifies zero output and sole ownership before releasing a driver stop latch.
Keep these dependencies together.
