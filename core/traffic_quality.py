"""Shared applicable-signal gates; optional lamp absence is not a fault."""

from core.scene_requirements import SIGNAL_SCENES
from core.validation import number
from core.interfaces import DecisionMode


def signal_stop_requirement(perception):
    """Nearest required route boundary across independent signal groups.

    None means all applicable groups permit proceeding; distance -1 means
    stopping is required but its boundary is unverified. These facts do not
    depend on the decision or on a scenario-specific speed/motion rule.
    """
    traffic = perception.traffic
    if getattr(traffic, 'signal_groups_valid', False) is True:
        groups = getattr(traffic, 'signal_groups', None)
        statuses = getattr(perception, 'source_status', {})
        status = statuses.get('traffic') if isinstance(statuses, dict) else None
        if isinstance(status, dict) and status.get('association_valid') is False:
            return dict(distance=-1., reason='signal_association_unavailable', signal_ids=[])
        transport_bad = (status is not None and (not isinstance(status, dict)
            or status.get('quality') in ('stale', 'regressed', 'unsynchronized', 'invalid')
            or status.get('read_ok') is False
            or (status.get('usable') is False and status.get('quality') not in
                ('signal_read_unavailable', 'signal_direction_unresolved', 'signal_stopline_unresolved',
                 'signal_record_invalid', 'signal_association_unavailable'))))
        if (not isinstance(groups, list) or len(groups) > 20000
                or (not groups and (traffic.required is True or traffic.observed is True))):
            return dict(distance=-1., reason='signal_groups_invalid', signal_ids=[])
        stopping = []
        for group in groups:
            if (not isinstance(group, dict) or not number(group.get('stop_line_distance'))
                    or group['stop_line_distance'] < -1
                    or (group['stop_line_distance'] < 0 and group['stop_line_distance'] != -1)
                    or group.get('association_valid') is not True
                    or type(group.get('valid')) is not bool
                    or type(group.get('ambiguous')) is not bool
                    or not isinstance(group.get('reason'), str)
                    or not isinstance(group.get('signal_ids'), list)
                    or not all(type(identity) is int for identity in group['signal_ids'])
                    or group.get('boundary_kind') not in ('stop_line', 'scope_entry', 'coverage_end', 'unknown')
                    or group.get('signal_state') not in
                    ('UNKNOWN', 'INVALID', 'RED', 'GREEN', 'YELLOW', 'RED_BLINK', 'GREEN_BLINK', 'YELLOW_BLINK', 'BLACK')):
                return dict(distance=-1., reason='signal_groups_invalid', signal_ids=[])
            if group['valid'] and (group['boundary_kind'] != 'stop_line' or group['stop_line_distance'] < 0):
                return dict(distance=-1., reason='signal_groups_invalid', signal_ids=[])
            if (transport_bad or group['valid'] is not True or group['ambiguous']
                    or group['signal_state'] != 'GREEN'):
                stopping.append(dict(distance=group['stop_line_distance'],
                    reason='signal_source_unusable' if transport_bad else group['reason'],
                    signal_ids=group['signal_ids'],
                    state_known=group['valid'] is True and not group['ambiguous'] and not transport_bad))
        return min(stopping, key=lambda item: item['distance']) if stopping else None
    # Old callers retain their single-signal contract, including unknown API
    # versus confirmed absence. A green without usable quality never releases.
    if not traffic_required(perception) or (traffic_usable(perception) and traffic.signal_state == 'GREEN'):
        return None
    statuses = getattr(perception, 'source_status', {})
    status = statuses.get('traffic') if isinstance(statuses, dict) else None
    distance = traffic.stop_line_distance
    known = (traffic.observed is True and number(distance) and distance >= 0
             and (status is None or (isinstance(status, dict)
                  and status.get('association_valid') is True)))
    return dict(distance=float(distance) if known else -1., reason=traffic.reason,
                signal_ids=[traffic.signal_id], state_known=traffic_usable(perception))


def signal_stop_bound(perception, front_offset=None, margin=0.0):
    requirement = signal_stop_requirement(perception)
    if requirement is None or requirement['distance'] < 0:
        return None
    if front_offset is not None and (not number(front_offset) or front_offset <= 0):
        return None
    if not number(margin) or margin < 0:
        return None
    return max(0., requirement['distance']-(front_offset or 0.)-margin)


def traffic_required(perception):
    traffic = getattr(perception, "traffic", None)
    if (getattr(traffic, "required", False) is True
            or getattr(traffic, "observed", False) is True):
        return True
    if (getattr(traffic, "association_valid", False) is True
            and getattr(traffic, "signal_presence", "unknown") == "absent"):
        return False
    return getattr(perception, "scene_id", 0) in SIGNAL_SCENES


def traffic_usable(perception):
    traffic = getattr(perception, "traffic", None)
    statuses = getattr(perception, "source_status", {})
    status = statuses.get("traffic") if isinstance(statuses, dict) else None
    if status is not None and (not isinstance(status, dict)
                               or status.get("usable") is not True
                               or status.get("read_ok") is False
                               or status.get("quality") in
                               ("stale", "regressed", "unsynchronized", "invalid")):
        return False
    return (getattr(traffic, "valid", False) is True
            and getattr(traffic, "observed", False) is True
            and getattr(traffic, "ambiguous", True) is False
            and getattr(traffic, "signal_state", "UNKNOWN") in ("RED", "YELLOW", "GREEN"))


def can_approach_signal(perception):
    """An unresolved lamp may be approached only towards a verified boundary.

    Legacy observed stop-line fields retain their contract. A grouped future
    scope boundary is distinct from a physical mapped stop line.
    """
    requirement = signal_stop_requirement(perception)
    return requirement is not None and requirement['distance'] >= 0


def bounded_signal_stop(perception, decision):
    """An unavailable lamp permits only a stop before its mapped boundary."""
    bound = signal_stop_bound(perception)
    return (bound is not None
            and getattr(decision, 'valid', False) is True
            and decision.mode in (DecisionMode.STOP, DecisionMode.EMERGENCY_BRAKE,
                                  DecisionMode.KEEP_LANE, DecisionMode.FOLLOW)
            and number(decision.stop_distance)
            and 0 <= decision.stop_distance <= bound)
