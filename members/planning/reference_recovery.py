"""Geometry-backed reference recovery; no assumed vehicle dimensions/capacity."""
import math

from core.geometry import normalize_angle, project_polyline
from core.validation import number

EPS = 1e-6


def capability(settings):
    values = (settings.front_offset_m, settings.half_width_m,
              settings.rear_offset_m, settings.wheelbase_m,
              settings.front_steer_max_rad)
    return all(number(value) and value > 0 for value in values)


def _inside_polygon(point, polygon):
    x, y = point
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        square = dx*dx+dy*dy
        ratio = max(0., min(1., ((x-a[0])*dx+(y-a[1])*dy)/square)) if square else 0.
        if math.hypot(x-a[0]-ratio*dx, y-a[1]-ratio*dy) <= EPS:
            return True
        if (a[1] > y) != (b[1] > y):
            intercept = a[0]+(y-a[1])*(b[0]-a[0])/(b[1]-a[1])
            if x < intercept:
                inside = not inside
    return inside


def _segment_distance(point, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    square = dx*dx+dy*dy
    ratio = max(0., min(1., ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/square)) if square else 0.
    return math.hypot(point[0]-a[0]-ratio*dx, point[1]-a[1]-ratio*dy)


def _straight_boundary(points):
    """Drop redundant knots only after proving the entire side is straight.

    Native straight roads can contain hundreds of collinear boundary knots.
    Retaining all of them in every swept-body polygon test wastes the frame
    budget. Monotone progress and sub-nanometre deviations preserve geometry;
    any turn, fold or non-monotone side keeps every supplied knot.
    """
    a, b = points[0], points[-1]
    dx, dy = b[0]-a[0], b[1]-a[1]
    square = dx*dx+dy*dy
    if square <= EPS*EPS:
        return points
    previous = 0.
    for point in points:
        ratio = ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/square
        if (ratio < previous or ratio > 1. or
                math.hypot(point[0]-a[0]-ratio*dx, point[1]-a[1]-ratio*dy) > 1e-9):
            return points
        previous = ratio
    return [a, b]


def _corridor(lane):
    left, right = lane.left_boundary, lane.right_boundary
    if left or right:
        if not isinstance(left, (list, tuple)) or not isinstance(right, (list, tuple)):
            raise ValueError('recovery lane boundaries malformed')
        if len(left) < 2 or len(right) < 2:
            raise ValueError('recovery requires both lane boundaries')
        left = [tuple(p[:2]) for p in left]
        right = [tuple(p[:2]) for p in right]
        polygon = left + list(reversed(right))
        if not all(len(p) == 2 and all(number(v) for v in p) for p in polygon):
            raise ValueError('recovery lane boundaries nonfinite')
        polygon = _straight_boundary(left) + list(reversed(_straight_boundary(right)))
        def contains(point):
            return _inside_polygon(point, polygon)
        contains.polygon = polygon
        return contains
    if lane.lane_width_valid is not True or not number(lane.lane_width) or lane.lane_width <= 0:
        raise ValueError('recovery requires verified lane boundaries or current-lane width')
    points = []
    for raw in lane.center_line:
        point = tuple(raw[:2])
        if not points or math.hypot(point[0]-points[-1][0], point[1]-points[-1][1]) > EPS:
            points.append(point)
    # A body check visits many nearby points. Index the finite-width strips,
    # rather than repeatedly projecting each body edge onto a full HDMap lane.
    # Long/complex segments retain a bounded fallback; this changes no geometry.
    half_width = lane.lane_width*.5
    cell_size = max(1., lane.lane_width)
    segments, cells, entries = [], {}, 0
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        segment = (a, dx, dy, dx*dx+dy*dy)
        index = len(segments)
        segments.append(segment)
        if cells is not None:
            xa, xb = (int(math.floor((value)/cell_size)) for value in
                      (min(a[0], b[0])-half_width, max(a[0], b[0])+half_width))
            ya, yb = (int(math.floor((value)/cell_size)) for value in
                      (min(a[1], b[1])-half_width, max(a[1], b[1])+half_width))
            entries += (xb-xa+1)*(yb-ya+1)
            if entries > 100000:
                cells = None
                continue
            for x in range(xa, xb+1):
                for y in range(ya, yb+1):
                    cells.setdefault((x, y), []).append(index)

    def distance(point):
        indices = (range(len(segments)) if cells is None else cells.get(
            (int(math.floor(point[0]/cell_size)), int(math.floor(point[1]/cell_size))), ()))
        best, best_index, best_raw = float('inf'), None, None
        for index in indices:
            a, dx, dy, square = segments[index]
            raw = ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/square
            ratio = max(0., min(1., raw))
            value = math.hypot(point[0]-a[0]-ratio*dx, point[1]-a[1]-ratio*dy)
            if value < best:
                best, best_index, best_raw = value, index, raw
        if (best_index is None or (best_index == 0 and best_raw < -EPS)
                or (best_index == len(segments)-1 and best_raw > 1+EPS)):
            return None
        return best

    def contains(point):
        value = distance(point)
        return value is not None and value <= half_width+EPS
    contains.polygon = None
    contains.width = lane.lane_width
    contains.distance = distance
    return contains


def _body_inside(corners, contains, pose_allowance=0.):
    if not all(contains(point) for point in corners):
        return False
    polygon=contains.polygon
    for a,b in zip(corners,corners[1:]+corners[:1]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        if polygon is not None:
            for c,d in zip(polygon,polygon[1:]+polygon[:1]):
                ux,uy=d[0]-c[0],d[1]-c[1]
                determinant=dx*uy-dy*ux
                if abs(determinant) <= EPS:
                    continue
                first=((c[0]-a[0])*uy-(c[1]-a[1])*ux)/determinant
                second=((c[0]-a[0])*dy-(c[1]-a[1])*dx)/determinant
                if EPS < first < 1-EPS and EPS < second < 1-EPS:
                    return False
                if pose_allowance > 0 and min(_segment_distance(a, c, d),
                        _segment_distance(b, c, d), _segment_distance(c, a, b),
                        _segment_distance(d, a, b)) < pose_allowance-EPS:
                    return False
        else:
            # Distance to a closed polyline is 1-Lipschitz. The half-step
            # allowance bounds every unsampled body-edge point as well.
            length=math.hypot(dx,dy)
            count=max(1,int(math.ceil(length/.1)))
            allowance=length/count*.5
            for j in range(count+1):
                point=(a[0]+dx*j/count,a[1]+dy*j/count)
                distance = contains.distance(point)
                if (distance is None or
                        distance+allowance+pose_allowance > contains.width*.5+EPS):
                    return False
    return True


def headings(points):
    result = []
    for i, point in enumerate(points):
        a = points[max(0, i-1)]
        b = points[min(len(points)-1, i+1)]
        result.append(math.atan2(b[1]-a[1], b[0]-a[0]))
    return result


def curvature(points, i):
    if i == 0:
        i = 1
    if i >= len(points)-1:
        i = len(points)-2
    a, b, c = points[i-1:i+2]
    ab, bc, ac = (math.hypot(b[0]-a[0], b[1]-a[1]),
                  math.hypot(c[0]-b[0], c[1]-b[1]),
                  math.hypot(c[0]-a[0], c[1]-a[1]))
    if min(ab, bc, ac) <= EPS:
        return 0.
    return 2*((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/(ab*bc*ac)


def check_path(points, lane, settings, check_until=None, initial_heading=None):
    """Full configured body inside known corridor; steering curvature feasible."""
    if not capability(settings):
        raise ValueError('reference recovery requires explicit front/rear/width and steering capability')
    if len(points) < 3:
        raise ValueError('recovery geometry too short')
    contains = _corridor(lane)
    orientations = headings(points)
    if initial_heading is not None:
        orientations[0] = initial_heading
    max_curvature = math.tan(settings.front_steer_max_rad)/settings.wheelbase_m
    radius = math.hypot(max(settings.front_offset_m, settings.rear_offset_m), settings.half_width_m)

    def body(pose, allowance=0.):
        x, y, angle = pose
        c, s = math.cos(angle), math.sin(angle)
        corners=[]
        for longitudinal,lateral in ((-settings.rear_offset_m,-settings.half_width_m),
            (settings.front_offset_m,-settings.half_width_m),
            (settings.front_offset_m,settings.half_width_m),
            (-settings.rear_offset_m,settings.half_width_m)):
            corners.append((x+longitudinal*c-lateral*s,y+longitudinal*s+lateral*c))
        return _body_inside(corners, contains, allowance+settings.lateral_margin_m)

    checks = [0]

    def swept(a, b, depth=0):
        # For linear rear-axle/heading interpolation, every body point is
        # within this displacement of its midpoint pose. Certify that whole
        # displacement fits; subdivide only where the corridor is close.
        turn = normalize_angle(b[2]-a[2])
        midpoint = ((a[0]+b[0])*.5, (a[1]+b[1])*.5, a[2]+turn*.5)
        displacement = (math.hypot(b[0]-a[0], b[1]-a[1])+radius*abs(turn))*.5
        checks[0] += 1
        if checks[0] > 20000:
            raise ValueError('reference recovery corridor-check budget exceeded')
        if body(midpoint, displacement):
            return True
        if depth >= 14 or displacement <= 1e-4:
            return False
        return swept(a, midpoint, depth+1) and swept(midpoint, b, depth+1)

    if not body((points[0][0], points[0][1], orientations[0])):
        raise ValueError('reference recovery vehicle footprint leaves verified lane corridor')
    distance = 0.
    for i, point in enumerate(points):
        if abs(curvature(points, i)) > max_curvature+EPS:
            raise ValueError('reference recovery curvature exceeds configured steering capability')
        if i == len(points)-1:
            break
        length = math.hypot(points[i+1][0]-point[0], points[i+1][1]-point[1])
        fraction = (1. if check_until is None else min(1., max(0., (check_until-distance)/length)))
        if fraction <= EPS:
            break
        turn = normalize_angle(orientations[i+1]-orientations[i])
        a = (point[0], point[1], orientations[i])
        b = (point[0]+fraction*(points[i+1][0]-point[0]),
             point[1]+fraction*(points[i+1][1]-point[1]), orientations[i]+fraction*turn)
        if not swept(a, b):
            raise ValueError('reference recovery vehicle footprint leaves verified lane corridor')
        distance += length*fraction
        if fraction < 1. or (check_until is not None and distance >= check_until-EPS):
            break
    return True


def _circle(a,b,c):
    ax,ay=a; bx,by=b; cx,cy=c
    determinant=2*(ax*(by-cy)+bx*(cy-ay)+cx*(ay-by))
    if abs(determinant) <= 1e-9:
        return None
    aa,bb,cc=ax*ax+ay*ay,bx*bx+by*by,cx*cx+cy*cy
    x=(aa*(by-cy)+bb*(cy-ay)+cc*(ay-by))/determinant
    y=(aa*(cx-bx)+bb*(ax-cx)+cc*(bx-ax))/determinant
    radius=math.hypot(ax-x,ay-y)
    return x,y,radius


def smooth_sparse(points, current_count, lane, ego, settings):
    """Reconstruct supported circular road samples, never an isolated bad join.

    Two neighbouring triples must describe the same bend within the observed
    available corridor. A true kink with no such support is still rejected.
    Every reconstructed vehicle pose is checked against the supplied corridor.
    """
    source_heading=[math.atan2(b[1]-a[1],b[0]-a[0]) for a,b in zip(points,points[1:])]
    source_s=[0.]
    for a,b in zip(points,points[1:]):
        source_s.append(source_s[-1]+math.hypot(b[0]-a[0],b[1]-a[1]))
    anchor=project_polyline(points[:current_count],ego.x,ego.y)
    # Avoid requiring geometry of an irrelevant sharp corner far downstream.
    active=[i for i in range(1,len(points)-1)
            if abs(normalize_angle(source_heading[i]-source_heading[i-1])) > settings.max_corner_angle
            and anchor['s']-(settings.rear_offset_m or 0) <= source_s[i] <= anchor['s']+settings.horizon]
    if not active:
        return points,current_count,False
    if not capability(settings):
        raise ValueError('sharp reference corner requires explicit steering and vehicle geometry for reconstruction')
    if lane.lane_width_valid is not True or not number(lane.lane_width):
        raise ValueError('sparse curve reconstruction requires verified current-lane width')
    allowance=lane.lane_width*.5-settings.half_width_m-settings.lateral_margin_m
    if allowance <= 0:
        raise ValueError('sparse curve has no verified vehicle corridor')
    circles={i:_circle(*points[i-1:i+2]) for i in range(1,len(points)-1)}
    selected={}
    for i in active:
        fit=circles[i]
        neighbours=[circles[j] for j in (i-1,i+1) if j in circles and circles[j] is not None]
        if fit is None or not any(math.hypot(fit[0]-other[0],fit[1]-other[1])+
                                  abs(fit[2]-other[2]) <= allowance for other in neighbours):
            raise ValueError('sharp reference corner has no coherent neighbouring road curvature')
        if i >= current_count-1:
            raise ValueError('sparse successor curve corridor unavailable')
        selected[i-1]=fit
        selected[i]=fit
    result=[points[0]]
    new_current_count=1
    for i,(a,b) in enumerate(zip(points,points[1:])):
        fit=selected.get(i)
        if fit is not None:
            x,y,radius=fit
            angle=math.atan2(a[1]-y,a[0]-x)
            turn=normalize_angle(math.atan2(b[1]-y,b[0]-x)-angle)
            count=max(2,int(math.ceil(abs(turn)*radius/min(settings.spacing,.5))))
            segment=[]
            for j in range(1,count+1):
                theta=angle+turn*j/count
                segment.append((x+radius*math.cos(theta),y+radius*math.sin(theta)))
            segment[-1]=b
            result.extend(segment)
        else:
            result.append(b)
        if i+1 < current_count:
            new_current_count=len(result)
    # Validate the vehicle's reachable section, not a pose at the map origin
    # which the ego has already passed (its rear may be outside that origin).
    current=result[:new_current_count]
    projected=project_polyline(current,ego.x,ego.y)
    checked=[projected['point']]
    for point in current[projected['index']+1:]:
        if math.hypot(point[0]-checked[-1][0],point[1]-checked[-1][1])>EPS:
            checked.append(point)
    check_path(checked,lane,settings,check_until=settings.horizon,
               initial_heading=ego.heading)
    return result,new_current_count,True


def recovery_minimum(offset, heading_error, speed, settings):
    """Length bound shared by comfort admission and actual candidate search."""
    return max(2*settings.front_offset_m, 2*settings.wheelbase_m,
        math.sqrt((10*math.sqrt(3)/3)*abs(offset)*speed**2/settings.lateral_acceleration),
        6*abs(math.tan(heading_error))*speed**2/settings.lateral_acceleration)


def lateral_recovery(reference, lane, ego, settings):
    """Quintic offset blend, body/curvature checked before accepting a candidate."""
    points=[point for _,point in reference]
    origin=points[0]
    angle=math.atan2(points[1][1]-origin[1],points[1][0]-origin[0])
    offset=-(ego.x-origin[0])*math.sin(angle)+(ego.y-origin[1])*math.cos(angle)
    slope=math.tan(normalize_angle(ego.heading-angle))
    available=min(reference[-1][0],settings.horizon)
    if not capability(settings):
        raise ValueError('lateral recovery requires explicit vehicle/steering capability')
    # Search increasingly gradual blends, using geometry checks to select the
    # length instead of treating a fixed lateral error as physical failure.
    # Bound the blend's added lateral acceleration as well as steering/body
    # feasibility. The quintic offset basis has max |d2/du2| = 10*sqrt(3)/3;
    # 6 conservatively bounds the heading-slope basis. A higher-speed small
    # offset needs a longer return, rather than manufacturing a tight curve
    # that the downstream speed profile can only handle by an emergency stop.
    minimum=recovery_minimum(offset, normalize_angle(ego.heading-angle), ego.speed, settings)
    candidates=[minimum*factor for factor in (1.,1.5,2.,3.,4.,6.)]
    for length in candidates:
        if length > available-settings.front_offset_m:
            continue
        values=[i*.25 for i in range(int(length/.25)+1)]+[length]
        values.extend(s for s,_ in reference if s>length+EPS)
        result=[]
        cursor=0
        for s in values:
            while cursor+1<len(reference)-1 and reference[cursor+1][0] < s:
                cursor+=1
            sa,a=reference[cursor]; sb,b=reference[cursor+1]
            ratio=(s-sa)/(sb-sa)
            x,y=a[0]+ratio*(b[0]-a[0]),a[1]+ratio*(b[1]-a[1])
            heading=math.atan2(b[1]-a[1],b[0]-a[0])
            u=min(1.,s/length)
            blend=offset*(1-10*u**3+15*u**4-6*u**5)
            blend+=slope*length*(u-6*u**3+8*u**4-3*u**5)
            point=(x-blend*math.sin(heading),y+blend*math.cos(heading))
            if not result or math.hypot(point[0]-result[-1][0],point[1]-result[-1][1])>EPS:
                result.append(point)
        result[0]=(ego.x,ego.y)
        try:
            check_path(result,lane,settings,check_until=length,initial_heading=ego.heading)
        except ValueError:
            continue
        return result,length
    raise ValueError('no vehicle/corridor-feasible lateral recovery within known reference')
