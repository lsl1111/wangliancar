"""R06 consumer for existing D03 policies; never invent a maneuver intent."""
from core.maneuver_facts import ManeuverFacts
from core.behavior_contract import require
from members.decision.behaviors.observations import Evidence, MotionObject
from members.decision.behaviors.lane_change import LaneCandidate


def lane_candidates(perception, frame, clock=None):
    require(frame.context.key()[:3] == (perception.case_id, perception.task_id, perception.scene_id)
            and frame.frame_id == perception.frame_id, 'MANEUVER_DECISION_CONTEXT_MISMATCH')
    facts = ManeuverFacts(perception, clock)
    require(frame.current(facts.check_current()) and not frame.paused, 'MANEUVER_DECISION_FRAME_UNUSABLE')
    objects = [MotionObject(v['id'], v['x'], v['y'], v['vx'], v['vy'], v['length'], v['width'], v['heading'])
               for v in facts.objects()]
    candidates, failures = [], []
    for lane_id in facts.neighbors():
        try:
            road, crossing = facts.road(lane_id), facts.crossing(lane_id)
            source = next(v for v in facts.env.neighbor_lanes if v['lane_id'] == lane_id)
            evidence = road['evidence']
            candidate = LaneCandidate(lane_id, crossing['side'], [p[:2] for p in road['center_line']],
                source.get('width_m'), 'BROKEN', bool(crossing['permitted_fragments']), True,
                Evidence(frame.context, evidence['frame_id'], evidence['observed_at_s'], evidence['valid_until_s'],
                    usable=True, coverage_verified=True, source=evidence['source'], source_kind=evidence['source_kind']),
                True, objects, [p[:2] for p in road['left_boundary']], [p[:2] for p in road['right_boundary']],
                [p[:2] for p in crossing['shared_boundary_world']],
                [[p[:2] for p in line] for line in crossing['permitted_fragments']])
            facts.check_current(True)
            candidates.append(candidate)
        except (ValueError, TypeError, KeyError, IndexError, OverflowError) as error:
            failures.append(dict(lane_id=lane_id, reason=str(error)))
    facts.check_current(True)
    return candidates, failures
