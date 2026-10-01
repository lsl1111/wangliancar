"""Fixed decision entry. Replace only the engine behind decide()."""

from members.decision.engine import DecisionEngine

DECISION_VERSION = "decision-speed-launch-v2"
_ENGINE = DecisionEngine()


def reset_decision(settings=None):
    """Start a new runtime session without carrying prior blind-stop state."""
    global _ENGINE
    _ENGINE = DecisionEngine(settings)


def decision_info():
    """Expose the running implementation and tuning for launcher diagnostics."""
    return {"version": DECISION_VERSION, "engine": type(_ENGINE).__name__,
            "cruise_speed": float(_ENGINE.settings.cruise_speed)}


def decide(perception):
    """Choose a behaviour for one perception frame.

    Input:  Perception (captain)
    Output: DecisionTarget (planning member)

    The engine is module-level so its anti-flapping state survives between
    frames. See members/decision/BASELINE.md for the behaviour table and the
    parameters that are still provisional.
    """
    return _ENGINE.run(perception)
