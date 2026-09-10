# Verification

Install `requirements-test.txt` in a Windows Python with Tk, then run from root:

```powershell
py -3 tools/check_project.py --tests
```

Native tests use mocked ROS, synthetic images, local HTTP servers and Tk windows.
`test_ros_transport.py` is a separate executable requiring sourced Noetic; do not
include it in native unittest discovery. Use `tools/isolated_bench_check.py
--execute --target local` for a disposable local ROS run or `--target robot` for
the installed ARM64 runtime. Both isolate synthetic topics from hardware.

Interactive app tests: start `tools/interactive_bench.py`, then run
`tools/exercise_interactive_bench.py` in another shell. Do not run another app
client concurrently, because heartbeat ownership is intentionally exclusive.

`desktop_sandbox.py`, `replay_steering.py`, and `evaluate_local_chat.py` are retained
test/research helpers. A green synthetic test does not validate physical motion.
See [TESTING](../docs/TESTING.md) and [bench results](../docs/BENCH_CHAT_CHECKS.md).
