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
