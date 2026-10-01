"""Fixed decision entry. Replace only the engine behind decide()."""

from members.decision.engine import DecisionEngine

_ENGINE = DecisionEngine()


def decide(perception):
    """Choose a behaviour for one perception frame.

    Input:  Perception (captain)
    Output: DecisionTarget (planning member)

    The engine is module-level so its anti-flapping state survives between
    frames. See members/decision/BASELINE.md for the behaviour table and the
    parameters that are still provisional.
    """
    return _ENGINE.run(perception)
