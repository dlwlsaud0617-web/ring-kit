# support_v2.py — 학습 규칙 적용 반지(+헤드) 서포트 전체 자동 생성
# 사용: python3 support_v2.py 입력.3dm 출력.3dm  (보이는 객체만 모델로 사용)
import sys, math, json, numpy as np, rhino3dm as r, trimesh
from collections import Counter
from shapely.geometry import Polygon, Point, box
from shapely.ops import unary_union
sys.path.insert(0, '/home/user/ring-kit')
from pipe_rules import pipe, max_degree
src, dst = sys.argv[1], sys.argv[2]
f = r.File3dm.Read(src)
Lname = {i: l.FullPath for i, l in enumerate(f.Layers)}
S = next((i for i, n in Lname.items() if n == 'S'), None)
if S is None:
    L = r.Layer(); L.Name = 'S'; L.Color = (0, 0, 255, 255); S = f.Layers.Add(L)

def mesh_of(g):
    V, F, off = [], [], 0
    for fc in g.Faces:
        m = fc.GetMesh(r.MeshType.Any)
        if not m or len(m.Vertices) == 0: continue
        v = np.array([[p.X, p.Y, p.Z] for p in m.Vertices])
        for i in range(len(m.Faces)):
            q = m.Faces[i]; F.append([q[0]+off, q[1]+off, q[2]+off])
            if q[2] != q[3]: F.append([q[0]+off, q[2]+off, q[3]+off])
        V.append(v); off += len(v)
    return trimesh.Trimesh(np.vstack(V), np.array(F), process=False)

vis = [o for o in f.Objects if isinstance(o.Geometry, r.Brep) and o.Attributes.Visible and f.Layers[o.Attributes.LayerIndex].Visible and o.Attributes.LayerIndex != S]
model = trimesh.util.concatenate([mesh_of(o.Geometry) for o in vis])
lo, hi = model.bounds
prev, log, feet = [], [], []

def add(b, tm, tag):
    a = r.ObjectAttributes(); a.LayerIndex = S; f.Objects.Add(b, a); prev.append(tm); log.append(tag)

def hit(o, d, maxd=1e9):
    loc, _, _ = model.ray.intersects_location([o], [d])
    if not len(loc): return None
    dist = np.linalg.norm(loc - o, axis=1); k = np.argmin(dist)
    return loc[k] if dist[k] <= maxd else None

EMB = 0.3
UP = np.array([0, 0, 1.0])
PT = lo[2] - 4.0                     # 바닥판 윗면: 모델 최저점 4mm 아래
XO = max(-lo[0], hi[0])              # 밴드 바깥 반폭
YB = 1.16                            # 밴드 반폭(아래쪽)

def post(x, y, rad, tag, tip_rad=None, tip_len=1.2):
    """바닥판→위로 첫 모델면. 바닥 끝 평평, 위 끝 둥글게 0.3 박힘"""
    h = hit(np.array([x, y, PT - 1]), UP)
    if h is None: return None
    if tip_rad is None:
        b, tm = pipe([x, y, PT - 0.3], h + UP * (EMB - rad), rad, flat_start=True); add(b, tm, tag)
    else:
        mid = h - UP * tip_len
        b, tm = pipe([x, y, PT - 0.3], mid, rad, flat_start=True); add(b, tm, tag)
        b, tm = pipe(mid, h + UP * (EMB - tip_rad), tip_rad); add(b, tm, tag + ' 접점')
    feet.append((x, y, rad)); return h

# 1) 바닥 기둥 5개 (가운데 Ø2.0, 나머지 Ø1.4)
for fx in (0, 0.38, -0.38, 0.67, -0.67):
    post(fx * XO, 0, 1.0 if fx == 0 else 0.7, '바닥 기둥')

# 2) 옆 기둥 4개 (밴드 양옆, y±0.9) + 가장자리 접점
col_top = -0.87
for sx in (1, -1):
    xc = sx * (XO + 0.95)
    for y in (-0.9, 0.9):
        b, tm = pipe([xc, y, PT - 0.3], [xc, y, col_top], 0.7, flat_start=True); add(b, tm, '옆 기둥')
        feet.append((xc, y, 0.7))
        for dz in (0.7, 2.7, 4.7, 6.7):
            o = np.array([xc, y, col_top - dz]); h = hit(o, np.array([-sx, 0, 0]), 2.5)
            if h is not None:
                b, tm = pipe(o, h + np.array([-sx * EMB, 0, 0]), 0.175); add(b, tm, '옆 접점')
        # 맨 위: 적도 근처로 비스듬히 (사용자 가이드 커브 방향)
        tgt = np.array([sx * (XO - 0.7), y * 0.5, 0.31]); o = np.array([xc, y, col_top])
        d = (tgt - o) / np.linalg.norm(tgt - o); h = hit(o, d, 3.5)
        if h is not None:
            b, tm = pipe(o, h + d * EMB, 0.175); add(b, tm, '옆 윗접점')

# 3) 안쪽 트리: 몸통 Ø1.0 + 가지 4 (Ø0.55) + 접점 Ø0.35
ctr = np.array([0, 0, (lo[2] + hi[2]) / 2 * 0 + 0.0])
ib = hit(np.array([0, 0, 0.0]), -UP)
b, tm = pipe(ib - UP * 0.3, [0, 0, -0.5], 0.5); add(b, tm, '트리 몸통')
fork = np.array([0, 0, -1.5])
for deg in (38, 64):
    for sx in (1, -1):
        d = np.array([sx * math.cos(math.radians(deg)), 0, math.sin(math.radians(deg))])
        h = hit(np.array([0, 0, 0.0]), d)
        if h is None: continue
        t0 = h - d * 1.0
        b, tm = pipe(fork, t0, 0.275); add(b, tm, '트리 가지')
        b, tm = pipe(t0, h + d * EMB, 0.175); add(b, tm, '트리 접점')

# 4) 헤드 테두리 아래: 밴드 밖으로 나온 모서리 4곳에 긴 기둥 Ø1.0 + 접점 Ø0.35
rails = [mesh_of(o.Geometry) for o in vis]
rails = [m_ for m_ in rails if m_.bounds[0][2] > 9 and (m_.bounds[1][2] - m_.bounds[0][2]) < 2.5 and (m_.bounds[1][0] - m_.bounds[0][0]) > 4]
hlo, hhi = rails[0].bounds
for sx in (1, -1):
    for sy in (1, -1):
        x, y = sx * (hhi[0] - 0.45), sy * (hhi[1] - 0.45)
        h = hit(np.array([x, y, PT - 1]), UP)
        if h is not None and h[2] > 9:
            post(x, y, 0.5, '헤드 기둥', tip_rad=0.175, tip_len=1.5)

# 5) 바닥판: 기둥발 + 여유 0.4, 밴드 줄은 하나로 잇기 (너무 넓지 않게)
shapes = [box(x - r_ - 0.4, y - r_ - 0.4, x + r_ + 0.4, y + r_ + 0.4) for x, y, r_ in feet]
xs = [x for x, y, r_ in feet]; strip_y = 1.0 + 0.4
shapes.append(box(min(xs), -strip_y, max(xs), strip_y))
for x, y, r_ in feet:      # 떨어진 발은 띠까지 연결
    if abs(y) > strip_y: shapes.append(box(x - 0.6, min(0, y), x + 0.6, max(0, y)))
poly = unary_union(shapes).simplify(0.05)
if poly.geom_type != 'Polygon': poly = poly.convex_hull
cs = list(poly.exterior.coords)
crv = r.PolylineCurve([r.Point3d(x, y, PT) for x, y in cs])
ext = r.Extrusion.Create(crv, -0.3, True)
a = r.ObjectAttributes(); a.LayerIndex = S; f.Objects.Add(ext, a); log.append('바닥판')
pl = trimesh.creation.extrude_polygon(poly, 0.3); pl.apply_translation((0, 0, PT - 0.3)); prev.append(pl)
eb = ext.GetBoundingBox()
if abs(eb.Max.Z - PT) > 1e-3:
    f.Objects.Delete(f.Objects[len(f.Objects) - 1].Attributes.Id)
    ext.Translate(r.Vector3d(0, 0, PT - eb.Max.Z)); a = r.ObjectAttributes(); a.LayerIndex = S; f.Objects.Add(ext, a)
    eb = ext.GetBoundingBox()
print('바닥판 z', round(eb.Min.Z, 2), round(eb.Max.Z, 2), '크기', round(poly.bounds[2]-poly.bounds[0], 2), 'x', round(poly.bounds[3]-poly.bounds[1], 2))

f.Write(dst, 7)
print(Counter(log))
bad = sum(1 for o in r.File3dm.Read(dst).Objects if o.Attributes.LayerIndex == S and isinstance(o.Geometry, r.Brep) and max_degree(o.Geometry) >= 3)
print('차수3:', bad)
sc = trimesh.Scene(); info = []
sc.add_geometry(model, node_name='m', geom_name='m'); info.append(dict(name='m', label='part', layer='기본값', vis=True, lc=[201,201,201], color=False))
for k, tm in enumerate(prev):
    sc.add_geometry(tm, node_name=f's{k}', geom_name=f's{k}'); info.append(dict(name=f's{k}', label='support', layer='S', vis=True, lc=[40,70,230], color=True))
base = dst[:-4]; sc.export(base + '.glb'); json.dump(info, open(base + '.json', 'w'), ensure_ascii=False)
