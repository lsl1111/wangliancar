"""Shared world-metre footprint calculations, independent of the SDK."""

import math

from core.validation import number


def longitudinal_extent(target, path_heading):
    """Half extent along the path; unknown orientation uses a bounding circle."""
    if not number(target.heading):
        return 0.5 * math.hypot(target.length, target.width)
    angle = target.heading - path_heading
    return 0.5 * (target.length * abs(math.cos(angle))
                  + target.width * abs(math.sin(angle)))


def footprint_entry(reference, target, lateral_padding):
    """First path contact with an oriented target and a lateral ego strip.

    Expand the target slabs by the strip projected onto its axes. This is a
    conservative enclosure of the Minkowski sum on bends/rotated objects,
    exact for aligned straight vehicles. It is not a swept turning-car model.
    The caller subtracts the rear-axle-to-front offset exactly once.
    """
    if not number(target.heading):
        return circle_entry(reference, target.x, target.y,
                            math.hypot(target.length, target.width)*0.5 + lateral_padding)
    c, s = math.cos(target.heading), math.sin(target.heading)
    for (sa, a), (sb, b) in zip(reference, reference[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            continue
        nx, ny = -dy/length, dx/length
        bounds = (target.length*0.5 + lateral_padding*abs(c*nx+s*ny),
                  target.width*0.5 + lateral_padding*abs(-s*nx+c*ny))
        fx, fy = a[0]-target.x, a[1]-target.y
        starts, slopes = (c*fx+s*fy, -s*fx+c*fy), (c*dx+s*dy, -s*dx+c*dy)
        lo, hi = 0.0, 1.0
        for origin, delta, bound in zip(starts, slopes, bounds):
            if abs(delta) <= 1e-9:
                if abs(origin) > bound + 1e-9:
                    hi = -1.0
                    break
            else:
                t1, t2 = (-bound-origin)/delta, (bound-origin)/delta
                lo, hi = max(lo, min(t1, t2)), min(hi, max(t1, t2))
        if lo <= hi + 1e-9:
            return sa + max(0.0, lo)*(sb-sa)
    return None


def circle_entry(reference, x, y, radius):
    for (sa, a), (sb, b) in zip(reference, reference[1:]):
        dx, dy, fx, fy = b[0]-a[0], b[1]-a[1], a[0]-x, a[1]-y
        aa = dx*dx+dy*dy
        if fx*fx+fy*fy <= radius*radius:
            return sa
        if aa <= 1e-9:
            continue
        bb, cc = 2*(fx*dx+fy*dy), fx*fx+fy*fy-radius*radius
        disc = bb*bb-4*aa*cc
        if disc < -1e-9:
            continue
        root = (-bb-math.sqrt(max(0.0, disc)))/(2*aa)
        if -1e-9 <= root <= 1+1e-9:
            return sa + max(0.0, min(1.0, root))*(sb-sa)
    return None
