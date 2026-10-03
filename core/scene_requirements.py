"""Minimum data channels needed by each known NEVC scene.

Unknown cases stay conservative. Optional observations may still inform safety
when they are fresh, but their absence is not a scene failure.
"""

TARGET_SCENES = frozenset(
    (1, 2, 3) + tuple(range(11, 19)) + tuple(range(21, 29))
    + tuple(range(30, 42))
)


def requires_targets(scene_id):
    if type(scene_id) is not int or not 1 <= scene_id <= 41:
        return True
    return scene_id in TARGET_SCENES


# Task semantics only; geometry/velocity still determine each conflict.
FOLLOW_SCENES = frozenset((14, 15, 22, 23, 24, 25))
AEB_SCENES = frozenset((1, 2, 3))
SIGNAL_SCENES = frozenset((10, 20, 37, 38))
