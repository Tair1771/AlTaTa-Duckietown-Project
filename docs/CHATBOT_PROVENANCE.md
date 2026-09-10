# Chatbot implementation and template verification

Reviewed 2026-09-10 from current source, saved authenticated robot metadata and
official Duckietown documentation. No new robot connection was attempted for
this review; the last attempt ended with a depleted battery.

## What this project implements

`laptop/duck2_companion.py` calls `laptop/live_chat.py`. The interpreter uses
Python string normalization, regular-expression rules and allowlisted intents.
The session manager validates turn queues with the shared directed map, sends
one instruction at a red stop and reconciles its acknowledgment with robot
status. `packages/duckie_lane_follower/src/duckie_lane_follower/live_session.py`
implements robot-side pause and speed-profile policy. Existing ROS gateways
deliver the commands to the controller. See [LIVE_CHAT](LIVE_CHAT.md).

The live chatbot was developed by writing rules and examples and exercising
parser/session, UI, gateway and controller tests. There is **no training dataset,
fitted model, fine-tuning procedure or online learning** in this implementation.
Its context is the explicit route, turn queue, current run and reported robot
state. It does not infer arbitrary typos or learn new phrases from conversations.

An earlier API-backed desktop app and a rejected local-model experiment are
retained as historical sources/tests. Neither powers the current live-chat UI.
Their evaluation is not training of the current chatbot.

## Was there an installed Duckietown chatbot template?

No chatbot template was identified in the material checked. The project's
earlier imported scaffold is recorded as `template-ros` in
[INTERFACE_REFERENCES](INTERFACE_REFERENCES.md). Duckietown's official
[project-template catalogue](https://docs.duckietown.com/ente/devmanual-software/beginner/dtproject/templates.html)
lists basic, ROS, core, documentation, learning-experience and dashboard
templates. Its [template-ros repository](https://github.com/duckietown/template-ros)
provides ROS project scaffolding. Those are not a natural-language command app.

The saved full inspection at 2026-09-10 07:44:25 UTC lists robot interface,
ROS commons, rosbridge, file/code APIs, proxy, dashboard, health/online and
Portainer services; it does not identify a chatbot service. The official
[robot-interface repository](https://github.com/duckietown/dt-duckiebot-interface)
describes the sensor and actuator drivers, without high-level functionality.
A dashboard or rosbridge connection alone is not a chatbot.

This supports **no identified preinstalled chatbot**, not proof that every file
on the SD card or every course-provided package has been searched. A separate
course/third-party template might exist. The friend's exact package name or
link would settle that narrower claim. Existing Duckietown attribution and
`LICENSE.pdf` remain intact; this project's chatbot builds on that ROS platform.
