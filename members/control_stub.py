"""Control member integration point.

The captain project intentionally returns invalid control so observe mode can never
move the vehicle by accident. The control member replaces compute_control().
"""

from core.interfaces import ControlOut


def compute_control(perception, trajectory):
    output = ControlOut()
    output.source = "captain_placeholder_no_actuation"
    output.valid = False
    return output

