"""실물 검출 좌표와 별개로 SDF 충돌 형상의 생성 전 겹침을 검사한다."""
import math
import xml.etree.ElementTree as ET

from ur5e_pick_place.object_spawner import model_file, rotate, rpy_to_quaternion


def _hull(points):
    points = sorted(set(points))
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    def half(items):
        out = []
        for p in items:
            while len(out) > 1 and cross(out[-2], out[-1], p) <= 0:
                out.pop()
            out.append(p)
        return out
    if len(points) < 3:
        raise ValueError('부품 충돌 형상의 면적이 없습니다.')
    return half(points)[:-1] + half(reversed(points))[:-1]


def _pose_points(points, pose):
    values = list(map(float, (pose or '0 0 0 0 0 0').split()))
    if len(values) != 6 or not all(map(math.isfinite, values)):
        raise ValueError('유효하지 않은 SDF pose')
    q = rpy_to_quaternion(*values[3:])
    return [tuple(v+o for v,o in zip(rotate(q,p),values[:3])) for p in points]


def collision_footprint(path):
    points = []
    root = ET.parse(path).getroot()
    for link in root.findall('./model/link'):
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            if geometry is None:
                raise ValueError(f'충돌 형상 없음: {path}')
            box = geometry.find('box')
            cylinder = geometry.find('cylinder')
            sphere = geometry.find('sphere')
            if box is not None:
                sizes = list(map(float,box.findtext('size').split()))
                if len(sizes) != 3 or not all(math.isfinite(v) and v > 0 for v in sizes):
                    raise ValueError('유효하지 않은 box 치수')
                vertices = [(sx*sizes[0]/2,sy*sizes[1]/2,sz*sizes[2]/2)
                            for sx in [-1,1] for sy in [-1,1] for sz in [-1,1]]
            elif cylinder is not None or sphere is not None:
                shape = cylinder if cylinder is not None else sphere
                radius = float(shape.findtext('radius'))
                length = float(cylinder.findtext('length')) if cylinder is not None else 2*radius
                if not all(math.isfinite(v) and v > 0 for v in [radius,length]):
                    raise ValueError('유효하지 않은 원기둥/구 치수')
                # 원을 내접 다각형으로 줄이면 작은 겹침을 놓친다. 외접 다각형으로 보수적으로 검사한다.
                radius /= math.cos(math.pi/32)
                vertices = [(radius*math.cos(2*math.pi*k/32),radius*math.sin(2*math.pi*k/32),z)
                            for k in range(32) for z in [-length/2,length/2]]
                if sphere is not None:
                    raise ValueError('구 충돌 형상은 아직 지원하지 않습니다.')
            else:
                raise ValueError(f'지원하지 않는 충돌 형상: {path}')
            vertices = _pose_points(vertices,collision.findtext('pose'))
            points.extend(_pose_points(vertices,link.findtext('pose')))
    if not points:
        raise ValueError(f'SDF에 충돌 형상이 없습니다: {path}')
    return _hull([(p[0],p[1]) for p in points])


def placed_footprint(request, footprint):
    if not all(math.isfinite(v) for v in [request.x,request.y,request.yaw_rad]):
        raise ValueError('부품 위치와 각도는 유한값이어야 합니다.')
    c,s = math.cos(request.yaw_rad),math.sin(request.yaw_rad)
    return [(request.x+c*x-s*y,request.y+s*x+c*y) for x,y in footprint]


def separating_gap(a, b):
    """분리축 검사. 음수면 겹침, 0이면 접촉. 이동 방향은 a에서 b 쪽이다."""
    best = (-math.inf,(1.,0.))
    for polygon in [a,b]:
        for p,q in zip(polygon,polygon[1:]+polygon[:1]):
            dx,dy = q[0]-p[0],q[1]-p[1]
            n = math.hypot(dx,dy)
            if n == 0:
                continue
            axis = (-dy/n,dx/n)
            pa = [x*axis[0]+y*axis[1] for x,y in a]
            pb = [x*axis[0]+y*axis[1] for x,y in b]
            forward,backward = min(pb)-max(pa),min(pa)-max(pb)
            gap,direction = (forward,axis) if forward >= backward else (backward,(-axis[0],-axis[1]))
            if gap > best[0]:
                best = gap,direction
    return best


def overlapping_pairs(requests, models_dir, clearance_m=.001):
    if not math.isfinite(clearance_m) or clearance_m < 0:
        raise ValueError('부품 사이 여유는 0 이상의 유한값이어야 합니다.')
    shapes = {r.part_type:collision_footprint(model_file(models_dir,r.part_type)) for r in requests}
    polygons = [placed_footprint(r,shapes[r.part_type]) for r in requests]
    pairs = []
    for i,a in enumerate(polygons):
        for j in range(i+1,len(polygons)):
            gap,direction = separating_gap(a,polygons[j])
            if gap < clearance_m-1e-9:
                pairs.append((i,j,gap,direction))
    return pairs


def assert_spawn_clear(requests, models_dir):
    pairs = overlapping_pairs(requests,models_dir)
    if pairs:
        details = '; '.join(f'{requests[i].name} ↔ {requests[j].name}: 충돌 형상 여유 {gap*1000:.2f} mm'
                            for i,j,gap,_ in pairs[:5])
        raise ValueError('가상 부품 겹침으로 생성하지 않았습니다. '+details+
                         '. 실제 부품 간격을 벌리거나 --screw-length-mm로 모델 길이를 맞춰 주세요.')
