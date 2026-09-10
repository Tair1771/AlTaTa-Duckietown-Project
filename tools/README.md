# Tools

Run Windows connection tools from the repository root with Windows Python.
Current commands and troubleshooting: [STARTUP](../docs/STARTUP.md).

| Files | Purpose |
| --- | --- |
| `inspect_robot_setup.py`, `Start-Duck2-Check.cmd` | Read-only SSH, ROS and ownership metadata |
| `setup_duck2_ssh.ps1` | One-time protected key/alias setup |
| `connect_companion.py`, `Start-Duck2-AppConnection.cmd` | Stationary preview and tunnel |
| `prepare_companion_driving.py`, `arm_driver_stopped.py`, `Start-Duck2-DrivingMode.cmd` | Stopped physical-controller preparation and verified driver arming |
| `run_duck2_ground_test.py`, `bounded_ground_supervisor.py`, `Start-Duck2-GroundTest.*` | Explicitly authorized bounded physical tests |
| `analyse_wheel_evidence.py` | Offline wheel/encoder log analysis |
| `isolated_bench_check.py` | Disposable network-none tests locally/onboard |
| `interactive_bench.py`, `bench_scene.py`, `bench_rpc.py` | Isolated onboard simulation and allowlisted stdio bridge |
| `exercise_interactive_bench.py` | Actual app callback checklist against simulation |
| `check_stationary_camera.py` | Preview decode/timestamp checks; no physical wheel commands |
| `build_local.py` | Pinned local AMD64/ARM64 build |
| `check_project.py` | Offline source/link/launcher audit and native tests |

`bench_scene.py` and `bench_rpc.py` are internal container helpers. Do not run
them on the robot host or against its real ROS master. Use their isolated runner.
Evidence belongs under local application data, not inside this folder.
