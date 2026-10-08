"""Shared polygon helpers lifted from the existing decision observations.

No sensing, behavior selection or SDK dependency. Touching is intersection.
"""
import math


def _finite(v):
    return type(v) in (int, float) and math.isfinite(v)


def cross(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])


def convex_polygon(raw):
    if (not isinstance(raw, (tuple, list)) or not 3 <= len(raw) <= 128
            or any(not isinstance(p, (tuple, list)) or len(p) != 2
                   or not all(_finite(v) for v in p) for p in raw)):
        raise ValueError("polygon coverage missing or invalid")
    points = [(float(p[0]), float(p[1])) for p in raw]
    if len(set(points)) != len(points):
        raise ValueError("repeated polygon vertex")
    area = sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(points, points[1:]+points[:1]))
    signs = [cross(points[i-1], points[i], points[(i+1)%len(points)])
             for i in range(len(points))]
    signs = [v > 0 for v in signs if abs(v) > 1e-8]
    if abs(area) <= 1e-8 or not signs or not all(v == signs[0] for v in signs):
        raise ValueError("nonconvex or degenerate polygon")
    # Consistent turn signs alone can admit a star with multiple windings.
    edges = list(zip(points, points[1:]+points[:1]))
    for i, (a,b) in enumerate(edges):
        for j, (c,d) in enumerate(edges[i+1:], i+1):
            if j == i+1 or (i == 0 and j == len(edges)-1):
                continue
            if segments_intersect(a,b,c,d):
                raise ValueError("self-intersecting polygon")
    return points


def simple_outline(raw, max_checks=100000):
    """Validate a possibly concave map outline without inventing a hull.

    Bound work for long SDK samples. Exhaustion is unknown geometry, never a
    certificate. Sorting edge boxes avoids quadratic work on straight lanes.
    """
    if (not isinstance(raw, (tuple, list)) or not 3 <= len(raw) <= 40000
            or any(not isinstance(p, (tuple, list)) or len(p) != 2
                   or not all(_finite(v) for v in p) for p in raw)):
        raise ValueError("invalid map outline")
    points = [tuple(p) for p in raw]
    if len(set(points)) != len(points):
        raise ValueError("repeated outline vertex")
    if abs(sum(cross(points[0], a, b) for a, b in zip(points, points[1:]+points[:1]))) <= 1e-8:
        raise ValueError("degenerate outline")
    boxes = sorted((min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1]), i, a, b)
                   for i, (a,b) in enumerate(zip(points, points[1:]+points[:1])))
    active, checks = [], 0
    for edge in boxes:
        active = [v for v in active if v[1]+1e-8 >= edge[0]]
        for prior in active:
            checks += 1
            if checks > max_checks:
                raise ValueError("outline validation budget exhausted")
            if abs(prior[4]-edge[4]) in (1, len(points)-1):
                continue
            if prior[3]+1e-8 < edge[2] or edge[3]+1e-8 < prior[2]:
                continue
            if segments_intersect(prior[5], prior[6], edge[5], edge[6]):
                raise ValueError("self-intersecting outline")
        active.append(edge)
    return points


def segments_intersect(a,b,c,d):
    signs = (cross(a,b,c), cross(a,b,d), cross(c,d,a), cross(c,d,b))
    if signs[0]*signs[1] < -1e-8 and signs[2]*signs[3] < -1e-8:
        return True
    for p,u,v,value in ((c,a,b,signs[0]), (d,a,b,signs[1]),
                         (a,c,d,signs[2]), (b,c,d,signs[3])):
        if (abs(value) <= 1e-8 and min(u[0],v[0])-1e-8 <= p[0] <= max(u[0],v[0])+1e-8
                and min(u[1],v[1])-1e-8 <= p[1] <= max(u[1],v[1])+1e-8):
            return True
    return False


def inside(point, region):
    values = [cross(a,b,point) for a,b in zip(region, region[1:]+region[:1])]
    return all(v >= -1e-8 for v in values) or all(v <= 1e-8 for v in values)


def intersects(left, right):
    for shape in (left, right):
        for a,b in zip(shape, shape[1:]+shape[:1]):
            nx,ny = -(b[1]-a[1]), b[0]-a[0]
            l = [nx*p[0]+ny*p[1] for p in left]
            r = [nx*p[0]+ny*p[1] for p in right]
            if max(l) < min(r)-1e-8 or max(r) < min(l)-1e-8:
                return False
    return True


def hull(points):
    ordered = sorted(set(points))
    if len(ordered) < 3:
        raise ValueError("insufficient swept footprint")
    lower,upper = [],[]
    for p in ordered:
        while len(lower) >= 2 and cross(lower[-2],lower[-1],p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2],upper[-1],p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]


def rectangle(x,y,heading,front,rear,half_width):
    c,s = math.cos(heading),math.sin(heading)
    return [(x+dx*c-dy*s,y+dx*s+dy*c) for dx,dy in
            ((front,half_width),(front,-half_width),(-rear,-half_width),(-rear,half_width))]


def circle_intersects(region,x,y,radius):
    if inside((x,y),region):
        return True
    for a,b in zip(region,region[1:]+region[:1]):
        dx,dy = b[0]-a[0],b[1]-a[1]
        length = dx*dx+dy*dy
        t = max(0.,min(1.,((x-a[0])*dx+(y-a[1])*dy)/length)) if length > 1e-12 else 0.
        if math.hypot(x-a[0]-t*dx,y-a[1]-t*dy) <= radius+1e-8:
            return True
    return False
