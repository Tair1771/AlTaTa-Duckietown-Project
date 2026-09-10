# Camera and connection

Windows OpenSSH forwards robot loopback services to the laptop:

| Service | Local address |
| --- | --- |
| Commands/status | `http://127.0.0.1:8765` |
| Normal, mask and overlay camera | `http://127.0.0.1:8766` |

For normal operation run `tools/Start-Duck2-DrivingMode.cmd`, then
`laptop/Start-Duck2Companion.cmd`. Preparation includes camera and SSH forwarding.
Do not start a second backend alongside it.

For stationary viewing only, run `tools/Start-Duck2-AppConnection.cmd`, then
the same companion. It prepares a driving-disabled `duck2-companion-preview`
container, which cannot run a route. Press Connect to duck2 and Start viewing.
Driving preparation stops the preview before claiming the same ports.

Preparation reuses a healthy tunnel and verifies both ports. Close an obsolete
project tunnel if it occupies ports without reaching its backend. Keep strict
host-key verification enabled. Camera-view controls do not stop the robot;
use STOP DUCK2 for movement. See [STARTUP](STARTUP.md) for recovery.
