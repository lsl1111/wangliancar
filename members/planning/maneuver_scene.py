"""Formal R06 snapshot for P02/P03 callers; no path or driving authority.

Road geometry and current complete visibility are separate constraints. Raw
object poses remain at source observation time; callers must supply a verified
prediction model and account for age, uncertainty and execution timing.
"""
from core.maneuver_facts import ManeuverFacts
from core.behavior_contract import require
from members.planning.candidate_validation import CorridorRegion


def read_maneuver_scene(perception, lane_ids, clock=None):
    require(isinstance(lane_ids, (list, tuple)) and 1 <= len(lane_ids) <= 3
            and all(isinstance(v, str) and v for v in lane_ids)
            and len(lane_ids) == len(set(lane_ids)), 'MANEUVER_PLANNING_LANE_SET_INVALID')
    facts = ManeuverFacts(perception, clock)
    allowed = [perception.lane.lane_id] + facts.neighbors()
    require(all(v in allowed for v in lane_ids), 'MANEUVER_PLANNING_LANE_UNAVAILABLE')
    objects, roads, crossings = facts.objects(), [], []
    for lane_id in lane_ids:
        roads.append(facts.road(lane_id, require_coverage=False))
        if lane_id != perception.lane.lane_id:
            crossings.append(facts.crossing(lane_id))
    raw = [p for road in roads for name in ('center_line', 'left_boundary', 'right_boundary') for p in road[name]]
    views = facts.coverage(raw)
    require(bool(views), 'MANEUVER_PLANNING_COVERAGE_UNAVAILABLE')
    deadlines = [facts.env.valid_until, facts.env.dynamic_valid_until]
    deadlines += [road['geometry_evidence']['valid_until_s'] for road in roads]
    deadlines += [v['valid_until_s'] for v in views]
    deadlines += [v['evidence']['valid_until_s'] for v in objects + crossings]
    facts.check_current(True)
    return dict(roads=roads, corridor=CorridorRegion([v['polygon'] for v in roads]),
                coverage=CorridorRegion([v['polygon'] for v in views]), coverage_records=views,
                objects=objects, crossings=crossings, map_digest=facts.map_digest(),
                source_observed_at_s=facts.env.dynamic_observed_at_s,
                source_valid_until_s=min(deadlines), frame_id=perception.frame_id,
                authority='geometry_and_current_visibility_only')
