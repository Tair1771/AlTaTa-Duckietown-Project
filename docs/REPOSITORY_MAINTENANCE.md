# Conservative cleanup — 2026-09-10

Before editing, a ZIP recovery snapshot of 142 project/report files was created
and its integrity checked outside the repository under local Duck2 backups.
The archive excludes Git metadata. No Git commands were used.

Removed only three obsolete duplicate entry shortcuts:

- `laptop/Start-Duck2Chat.cmd`
- `laptop/Start-OfflineChat.cmd`
- `laptop/Start-CommandPreview.cmd`

Their Python implementations and tests are retained and can be run directly;
see [laptop/README](../laptop/README.md). All four current companion/scenario/
simulation shortcuts remain, along with connection, driving preparation and
ground-diagnostic launchers. The active app imports transport from `chat_core.py`,
so that shared file was explicitly retained.

Runtime source, controller presets, calibration, ROS packaging, dependencies,
licence notices, test cases, experimental modules and diagnostic history were
not removed. Uncertain items were kept. Old bench setup is preserved in
[HISTORICAL_BENCH_SETUP](HISTORICAL_BENCH_SETUP.md); old app instructions are in
[LEGACY_APPS](../laptop/LEGACY_APPS.md). Current entry documents now lead to the
live companion rather than obsolete workflows.

The report source and PDF are in [report](../report/README.md). Its live-chat
description reflects implemented software; the battery-interrupted physical
attempt remains incomplete in [TESTING](TESTING.md). No motion, robot changes,
commits or uploads were performed for this refresh.


## Documentation-refresh verification — 2026-09-10

- The 77-source syntax, relative documentation-link and Windows launcher-target
  audit passed.
- 63 focused native tests passed across `test_live_chat`, `test_live_navigation`
  and `test_route_planner`; no hardware or live ROS master was used.
- The report compiled to four pages. Every rendered page was visually reviewed;
  its PDF text and LaTeX source passed the requested topic-exclusion check.
- Runtime Python, controller/ROS launchers, configuration, dependencies, tests
  and licence were byte-compared against the recovery snapshot and unchanged.
- Three obsolete shortcuts were the only removed files. The latest physical
  attempt is still incomplete; no physical test, robot change or Git operation
  was performed during this documentation refresh.
