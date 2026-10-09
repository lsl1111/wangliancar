"""Exact topology of a bounded union of explicit simple XY polygons.

This is geometry, not map/visibility authority. Every input polygon denotes
only its own interior; gaps, holes and disconnected components are retained.
Float inputs are treated as exact rational coordinates for topology. Exposed
edge endpoints return to float only for the existing continuous distance test,
with a conservative conversion/roundoff allowance subtracted from clearance.
"""
import heapq
import math
from fractions import Fraction

from core.region_geometry import simple_outline
from core.validation import number

MAX_POLYGONS=32
MAX_VERTICES=20000
MAX_FRACTION_BITS=8192


class NumericResolution(Exception):
    pass


def _bounded(value):
    if max(value.numerator.bit_length(),value.denominator.bit_length())>MAX_FRACTION_BITS:
        raise NumericResolution("REGION_NUMERIC_RESOLUTION")
    return value


def _cross(a,b): return _bounded(a[0]*b[1]-a[1]*b[0])
def _sub(a,b): return (a[0]-b[0],a[1]-b[1])
def _dot(a,b): return _bounded(a[0]*b[0]+a[1]*b[1])
def _at(edge,t): return tuple(_bounded(edge['a'][i]+t*edge['vector'][i]) for i in (0,1))


def _box(points):
    return (min(p[0] for p in points),min(p[1] for p in points),
            max(p[0] for p in points),max(p[1] for p in points))


def _box_gap(a,b):
    return math.hypot(max(0.,a[0]-b[2],b[0]-a[2]),max(0.,a[1]-b[3],b[1]-a[3]))


class _BodyBoxes(object):
    def __init__(self,body):
        self.body,self.box,self.origin,self.axes=body,_box(body),body[0],[]
        for a,b in zip(body,body[1:]+body[:1]):
            dx,dy=b[0]-a[0],b[1]-a[1]
            length=math.hypot(dx,dy)
            if not number(length) or length<=0: raise ValueError('invalid body edge')
            nx,ny=-dy/length,dx/length
            values=[(p[0]-self.origin[0])*nx+(p[1]-self.origin[1])*ny for p in body]
            if not all(number(v) for v in values): raise ValueError('body projection overflow')
            self.axes.append((nx,ny,min(values),max(values)))

    def lower_bound(self,box):
        lower=_box_gap(self.box,box)
        for nx,ny,low,high in self.axes:
            xmin,xmax=(box[0],box[2]) if nx>=0 else (box[2],box[0])
            ymin,ymax=(box[1],box[3]) if ny>=0 else (box[3],box[1])
            first=(xmin-self.origin[0])*nx+(ymin-self.origin[1])*ny
            last=(xmax-self.origin[0])*nx+(ymax-self.origin[1])*ny
            lower=max(lower,first-high,low-last)
        return lower

    def segment_gap(self,edge):
        # Separating-axis distance is a lower bound, sufficient for a road
        # clearance certificate. It avoids repeated vertex/edge distances.
        # Zero/unresolved separation is checked with exact contact below.
        a,b=edge['points']
        lower=0.
        for nx,ny,low,high in self.axes:
            first=(a[0]-self.origin[0])*nx+(a[1]-self.origin[1])*ny
            last=(b[0]-self.origin[0])*nx+(b[1]-self.origin[1])*ny
            if not number(first) or not number(last): raise ValueError('edge projection overflow')
            lower=max(lower,min(first,last)-high,low-max(first,last))
        nx,ny=edge['normal']
        values=[(p[0]-a[0])*nx+(p[1]-a[1])*ny for p in self.body]
        if not all(number(v) for v in values): raise ValueError('edge projection overflow')
        return max(lower,min(values),-max(values))


def _cut(edge,t):
    edge['cuts'].add(_bounded(t))
    if t==0: edge['start_contact']=True
    if t==1: edge['end_contact']=True


def _contacts(first,second):
    shared=False
    for t,a in ((0,first['a']),(1,first['b'])):
        for u,b in ((0,second['a']),(1,second['b'])):
            if a==b:
                _cut(first,Fraction(t)); _cut(second,Fraction(u)); shared=True
    r,s=first['vector'],second['vector']
    offset=_sub(second['a'],first['a'])
    denominator=_cross(r,s)
    same=(first['a']==second['a'] and first['b']==second['b'] or
          first['a']==second['b'] and first['b']==second['a'])
    if shared and (denominator or same): return True
    if denominator:
        t=_bounded(_cross(offset,s)/denominator)
        u=_bounded(_cross(offset,r)/denominator)
        if 0<=t<=1 and 0<=u<=1:
            _cut(first,t); _cut(second,u)
            return True
    elif _cross(offset,r)==0:
        square=_dot(r,r)
        low,high=sorted((_bounded(_dot(offset,r)/square),
                         _bounded(_dot(_sub(second['b'],first['a']),r)/square)))
        low,high=max(Fraction(0),low),min(Fraction(1),high)
        if low<=high:
            for t in (low,high):
                _cut(first,t)
                u=_bounded(_dot(_sub(_at(first,t),second['a']),s)/_dot(s,s))
                _cut(second,u)
            return True
    return shared


class _Polygon(object):
    def __init__(self,points,check):
        self.points=points
        self.box=_box(points)
        self.exact=[tuple(Fraction(v) for v in p) for p in points]
        area=Fraction(0)
        for a,b in zip(self.exact,self.exact[1:]+self.exact[:1]):
            check(); area=_bounded(area+_cross(_sub(a,self.exact[0]),_sub(b,self.exact[0])))
        if area==0: raise ValueError('region has no exact area')
        if area<0:
            self.points=list(reversed(points)); self.exact.reverse()
        self.edges=list(zip(self.exact,self.exact[1:]+self.exact[:1]))
        for index,b in enumerate(self.exact):
            check(); a,c=self.exact[index-1],self.exact[(index+1)%len(self.exact)]
            if _cross(_sub(a,b),_sub(c,b))==0 and _dot(_sub(a,b),_sub(c,b))>0:
                raise ValueError('exact adjacent region edges overlap')
        self.low,self.high=min(p[1] for p in self.exact),max(p[1] for p in self.exact)
        self.bands=[[] for unused in range(min(128,len(points)))]
        for edge in self.edges:
            a,b=edge
            coefficients=(a[1]-b[1],b[0]-a[0],_cross(a,b))
            denominator=max(v.denominator for v in coefficients)
            # Input int/float denominators are powers of two. Integer line
            # coefficients avoid repeated rational divisions at every query.
            integers=tuple(v.numerator*(denominator//v.denominator) for v in coefficients)
            record=(a,b,_sub(b,a),integers)
            for index in range(self.band(min(a[1],b[1])),self.band(max(a[1],b[1]))+1):
                check(); self.bands[index].append(record)

    def band(self,y):
        return max(0,min(len(self.bands)-1,int((y-self.low)*len(self.bands)/(self.high-self.low))))

    def state(self,point,check):
        # -1 outside, 1 strictly inside, or the oriented boundary edge.
        x,y=point
        if not self.box[0]<=x<=self.box[2] or not self.low<=y<=self.high: return -1
        denominator=x.denominator*y.denominator//math.gcd(x.denominator,y.denominator)
        if denominator.bit_length()>MAX_FRACTION_BITS: raise NumericResolution('REGION_NUMERIC_RESOLUTION')
        ix,iy=x.numerator*(denominator//x.denominator),y.numerator*(denominator//y.denominator)
        contained=False
        for a,b,vector,coefficients in self.bands[self.band(y)]:
            check()
            orientation=coefficients[0]*ix+coefficients[1]*iy+coefficients[2]*denominator
            if orientation.bit_length()>2*MAX_FRACTION_BITS: raise NumericResolution('REGION_NUMERIC_RESOLUTION')
            if (orientation==0 and min(a[0],b[0])<=x<=max(a[0],b[0])
                    and min(a[1],b[1])<=y<=max(a[1],b[1])):
                return vector
            if (a[1]>y)!=(b[1]>y):
                if (orientation>0)==(b[1]>a[1]): contained=not contained
        return 1 if contained else -1


def _tree(segments,check):
    check()
    box=(min(v['box'][0] for v in segments),min(v['box'][1] for v in segments),
         max(v['box'][2] for v in segments),max(v['box'][3] for v in segments))
    if len(segments)<=8: return (box,segments,None)
    axis=0 if box[2]-box[0]>=box[3]-box[1] else 1
    ordered=sorted(segments,key=lambda v:v['box'][axis]+(v['box'][axis+2]-v['box'][axis])*.5)
    middle=len(ordered)//2
    return (box,None,(_tree(ordered[:middle],check),_tree(ordered[middle:],check)))


def _exact_contact(body,edge,check):
    shapes=([tuple(Fraction(v) for v in p) for p in body],edge['exact'])
    for shape in shapes:
        for a,b in zip(shape,shape[1:]+shape[:1]):
            check(); vector=_sub(b,a)
            values=[[_cross(vector,_sub(p,a)) for p in vertices] for vertices in shapes]
            if max(values[0])<min(values[1]) or max(values[1])<min(values[0]): return False
    return True


class _PreparedRegion(object):
    def __init__(self,polygons,segments,roundoff,check):
        self.polygons,self.roundoff=polygons,roundoff
        self.root=_tree(segments,check)
        self.edge_count=len(segments)

    def gap(self,body,point,check):
        exact=tuple(Fraction(v) for v in point)
        if not any(poly.state(exact,check)!=-1 for poly in self.polygons): return -1.
        boxes=_BodyBoxes(body)
        queue=[(boxes.lower_bound(self.root[0]),0,self.root)]
        serial,minimum=0,float('inf')
        while queue:
            check()
            lower,unused,node=heapq.heappop(queue)
            if lower-self.roundoff>=minimum: break
            if node[1] is not None:
                for edge in node[1]:
                    check()
                    if boxes.lower_bound(edge['box'])-self.roundoff>=minimum: continue
                    gap=boxes.segment_gap(edge)
                    if gap<=self.roundoff:
                        if _exact_contact(body,edge,check): return 0.
                        raise NumericResolution('REGION_CLEARANCE_WITHIN_ROUNDOFF')
                    minimum=min(minimum,gap)
            else:
                for child in node[2]:
                    serial+=1
                    heapq.heappush(queue,(boxes.lower_bound(child[0]),serial,child))
        return minimum-self.roundoff


class CorridorRegion(object):
    """Explicit union input; constructing this object proves no source facts.

    Use a list of simple polygon XY rings. A ring may be concave. The caller
    must separately establish drivable area, complete visibility, permission,
    source identity and deadlines. A hull or guessed corridor is never added.
    """
    def __init__(self,polygons):
        self.polygons=polygons
        self._fingerprint,self._prepared=None,None

    def prepare(self,check,max_checks):
        raw=self.polygons
        if (not isinstance(raw,(tuple,list)) or not 1<=len(raw)<=MAX_POLYGONS
                or any(not isinstance(v,(tuple,list)) for v in raw)
                or sum(len(v) for v in raw)>MAX_VERTICES):
            raise ValueError('bounded explicit polygon regions required')
        snapshot=[]
        for ring in raw:
            values=[]
            for point in ring:
                check()
                if (not isinstance(point,(tuple,list)) or len(point)!=2
                        or not all(number(v) for v in point)):
                    raise ValueError('explicit finite region XY vertices required')
                values.append(tuple(point))
            snapshot.append(tuple(values))
        fingerprint=tuple(snapshot)
        if fingerprint==self._fingerprint and self._prepared is not None:
            check(); return self._prepared
        polygons=[]
        for ring in snapshot:
            check()
            points=simple_outline(ring,max_checks=max_checks,check_step=check)
            if not all(number(v) for p in points for v in p): raise ValueError('invalid region coordinates')
            extent=_box(points)
            if not number(extent[2]-extent[0]) or not number(extent[3]-extent[1]):
                raise ValueError('region arithmetic overflow')
            polygons.append(_Polygon(points,check))
        edges=[]
        for index,poly in enumerate(polygons):
            for edge_index,(a,b) in enumerate(poly.edges):
                check()
                edge=dict(polygon=index,a=a,b=b,vector=_sub(b,a),cuts={Fraction(0),Fraction(1)},
                          start_contact=False,end_contact=False,box=_box((a,b)),internal=False,index=edge_index)
                edges.append(edge)
        reversed_edges={}
        def point_key(point):
            # Fraction.__hash__ in Python 3.6 repeatedly computes a modular
            # inverse. Canonical numerator/denominator pairs are exact keys.
            return tuple((v.numerator,v.denominator) for v in point)
        for edge in edges:
            check()
            akey,bkey=point_key(edge['a']),point_key(edge['b'])
            for prior in reversed_edges.get((bkey,akey),[]):
                if prior['polygon']!=edge['polygon']:
                    prior['internal']=edge['internal']=True
            reversed_edges.setdefault((akey,bkey),[]).append(edge)
        active=[]
        for index in sorted(range(len(edges)),key=lambda i:edges[i]['box'][0]):
            check(); edge=edges[index]
            active=[i for i in active if edges[i]['box'][2]>=edge['box'][0]]
            for prior_index in active:
                check(); prior=edges[prior_index]
                same=prior['polygon']==edge['polygon']
                if same and abs(prior['index']-edge['index']) in (1,len(polygons[edge['polygon']].edges)-1):
                    continue
                if prior['box'][3]<edge['box'][1] or edge['box'][3]<prior['box'][1]: continue
                if _contacts(prior,edge) and same:
                    raise ValueError('exact region self intersection')
            active.append(index)
        segments,roundoff,cache,previous_polygon=[],0.,None,None
        for edge in edges:
            check()
            own=edge['polygon']
            if previous_polygon!=own: cache=None
            if edge['internal']:
                cache=None; previous_polygon=own
                continue
            cuts=sorted(edge['cuts'])
            for low,high in zip(cuts,cuts[1:]):
                if high==low: continue
                a,b=_at(edge,low),_at(edge,high)
                midpoint=tuple(_bounded((a[i]+b[i])/2) for i in (0,1))
                reuse=low==0 and not edge['start_contact'] and cache is not None
                states,internal={},False
                for other,poly in enumerate(polygons):
                    if other==own: continue
                    check()
                    state=cache.get(other) if reuse else None
                    if state is None: state=poly.state(midpoint,check)
                    states[other]=state if state in (-1,1) else None
                    if state==1:
                        internal=True
                    elif state!=-1 and _cross(edge['vector'],state)==0 and _dot(edge['vector'],state)<0:
                        internal=True  # The other interior covers our outward side.
                cache=states if high==1 and not edge['end_contact'] else None
                if internal: continue
                converted=[]
                for point in (a,b):
                    value=tuple(float(v) for v in point)
                    error=math.hypot(*(float(abs(point[i]-Fraction(value[i]))) for i in (0,1)))
                    # Include arithmetic roundoff of the subsequent distance
                    # and box tests; it can only reduce a clearance proof.
                    allowance=32.*(2.**-52)*max(1.,*(abs(v) for v in value))
                    roundoff=max(roundoff,error+allowance)
                    converted.append(value)
                if converted[0]==converted[1]:
                    raise NumericResolution('REGION_EDGE_COLLAPSED')
                dx,dy=converted[1][0]-converted[0][0],converted[1][1]-converted[0][1]
                length=math.hypot(dx,dy)
                if not number(length) or length<=0: raise ValueError('region edge arithmetic overflow')
                segments.append(dict(points=converted,exact=[a,b],box=_box(converted),normal=(-dy/length,dx/length)))
            previous_polygon=own
        if not segments: raise ValueError('region has no exterior boundary')
        prepared=_PreparedRegion(polygons,segments,roundoff,check)
        check()
        self._fingerprint,self._prepared=fingerprint,prepared
        return prepared
