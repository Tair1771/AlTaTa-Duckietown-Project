"""Small live route transaction layer; no chat or wheel-level commands."""


def start_route(transport, plan, cancelled=lambda: False, live_session=None):
    """Confirm connectivity, set one planned route, then release its stop."""
    if plan is None:
        raise ValueError("A complete route is required")
    status = transport.poll_status()
    initial_epoch = status.get("control_epoch")
    if isinstance(initial_epoch, bool) or not isinstance(initial_epoch, int):
        raise RuntimeError("Controller did not report a valid control epoch")
    if cancelled():
        return None
    if live_session is not None and (status.get("live_session") or {}).get("version") != 1:
        raise RuntimeError("Controller needs the live-chat update; keep duck2 stopped and prepare the updated image")
    if live_session is not None and (status.get("live_session") or {}).get("pause_mode") != "immediate":
        raise RuntimeError("Controller needs the immediate-pause update; prepare the updated driving mode before Start")
    publishers = status.get("wheel_publishers")
    if (not isinstance(publishers, list) or len(publishers) != 1
            or not publishers[0].endswith("/lane_follower_node")):
        raise RuntimeError(
            "Route control does not have exclusive wheel ownership; keep duck2 stopped")
    route = list(plan.route) if live_session is None else list(plan.route[:2])
    options = {} if live_session is None else {"managed_session": True, "run_id": live_session.run_id}
    if live_session is not None and live_session.stop_at_next_red:
        if (status.get("live_session") or {}).get("supports_pause_check") is not True:
            raise RuntimeError("Prepare the updated controller before the pause check")
        options["stop_at_next_red"] = True
    if live_session is not None and live_session.finish_approach is not None:
        if (status.get("live_session") or {}).get("supports_finish_approach") is not True:
            raise RuntimeError("Prepare the updated final-red-line controller before Start")
        options["finish_approach"] = live_session.finish_approach
    if live_session is not None and live_session.stop_after_junction:
        if (status.get("live_session") or {}).get("supports_stop_after_junction") is not True:
            raise RuntimeError("Prepare the single-crossing controller before Start")
        options["stop_after_junction"] = True
    if live_session is not None and (live_session.finish_after_junction_red or live_session.center_initial_straight):
        if (status.get("live_session") or {}).get("supports_initial_straight_check") is not True:
            raise RuntimeError("Prepare the initial-straight controller before Start")
        options["finish_after_junction_red"] = live_session.finish_after_junction_red
        options["center_initial_straight"] = live_session.center_initial_straight
    ack = transport.send(
        "set_route", route=route, map_id=plan.map_id,
        start_approach=plan.start_approach,
        destination_approach=(plan.destination_approach if live_session is None else plan.start_approach),
        position_confirmed=True, expected_control_epoch=initial_epoch, **options)
    if ack.get("accepted") is not True:
        raise RuntimeError(str(ack.get("reason", "Route was rejected")))
    if cancelled():
        return None
    status = transport.poll_status()
    if status.get("control_epoch") != initial_epoch:
        raise RuntimeError("A Stop superseded this Start request")
    if cancelled():
        return None
    ack = transport.send(
        "continue", expected_control_epoch=initial_epoch)
    if ack.get("accepted") is not True:
        raise RuntimeError(str(ack.get("reason", "Start was rejected")))
    return transport.poll_status()
