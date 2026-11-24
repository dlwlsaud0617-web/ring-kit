#!/usr/bin/env python3
"""
support_v3.py — 회사 정답(1124 티파니 반지 서포트) 방식 그대로 서포트 생성

정답 파일 실측 규칙
  바닥판  : '+' 모양 (가로 띠 y±1.5 + 헤드 기둥용 세로 띠), 두께 0.3, 모델 최저점 3.0 아래
  바닥 기둥: 4개 x=±0.295·R, ±0.59·R (R=밴드 바깥 반폭), Ø1.0, 곧게, 양끝 평평
  가운데   : 아래 몸통 Ø1.3 (판→밴드 바닥), 안쪽 몸통 Ø1.0 (밴드 안쪽 바닥→중심 -0.6)
  안쪽 가지: 4개 Ø0.96 곧게, 끝 둥글게, 중심(z-1.06)→안쪽 윗면 40°·72°  (접점 따로 없음)
  옆      : 양쪽 가운데 기둥 1개 Ø0.6 (x=R+1.24) — 아래 밴드 옆면 접점 없음
  파베 발  : 아래 4줄 × 앞뒤(y±0.5) → 갈고리 곡선 파이프 Ø0.3, 끝 둥글게
            기둥에서 수직으로 출발(발보다 1.8 아래) → 휘어서 발 아랫면에 수평으로 닿음
  헤드    : 앞뒤 기둥 2개 Ø1.0 (y=±0.67·헤드반길이) → 위에서 곡선 6갈래 Ø0.55 로 헤드 밑면
  차수    : 곧은 파이프 1(→ 여기선 2 이하), 곡선 파이프 3 유지
사용: python3 support_v3.py 입력.3dm 출력.3dm
"""
import sys, math, json, numpy as np, rhino3dm as r, trimesh
from collections import Counter
from shapely.geometry import box
from shapely.ops import unary_union
sys.path.insert(0, '/home/user/ring-kit')
from pipe_rules import pipe, max_degree

PLATE_GAP, PLATE_T = 3.0, 0.3
UP = np.array([0, 0, 1.0])
W45 = math.sqrt(0.5)


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


def curved_pipe(ctrl, rad, round_ends=True):
    """곡선 파이프 (경로 차수3, 단면 원 차수2). 끝은 반지름 0으로 오므려 둥글게 닫음"""
    C = [np.asarray(p, float) for p in ctrl]
    t0 = C[1] - C[0]; t0 /= np.linalg.norm(t0); t1 = C[-1] - C[-2]; t1 /= np.linalg.norm(t1)
    path, rads = [], []
    if round_ends:
        path += [C[0] - t0 * rad * 0.9, C[0] - t0 * rad * 0.9]; rads += [0.0, rad]
    path += C; rads += [rad] * len(C)
    if round_ends:
        path += [C[-1] + t1 * rad * 0.9, C[-1] + t1 * rad * 0.9]; rads += [rad, 0.0]
    n = len(path)
    # 각 제어점의 진행방향
    tang = []
    for i in range(n):
        a = path[max(i - 1, 0)]; b = path[min(i + 1, n - 1)]
        d = b - a
        if np.linalg.norm(d) < 1e-9: d = t0 if i < n / 2 else t1
        tang.append(d / np.linalg.norm(d))
    ref = np.array([0, 0, 1.0]) if abs(tang[0][2]) < 0.9 else np.array([1.0, 0, 0])
    s = r.NurbsSurface.Create(3, True, 4, 3, n, 9)
    prev_u = None
    for i in range(n):
        tt = tang[i]
        u = ref - tt * (ref @ tt) if prev_u is None else prev_u - tt * (prev_u @ tt)
        u /= np.linalg.norm(u); prev_u = u; v = np.cross(tt, u)
        for j in range(9):
            a = math.pi / 4 * j
            k = 1.0 if j % 2 == 0 else 1 / W45
            w = 1.0 if j % 2 == 0 else W45
            q = path[i] + rads[i] * k * (math.cos(a) * u + math.sin(a) * v)
            s.Points[i, j] = r.Point4d(q[0] * w, q[1] * w, q[2] * w, w)
    ku = [0, 0] + list(range(n - 2)) + [n - 3, n - 3]
    for i, kv in enumerate(ku): s.KnotsU[i] = kv
    for i, kv in enumerate([0, 0, 1, 1, 2, 2, 3, 3, 4, 4]): s.KnotsV[i] = kv
    brep = r.Brep.CreateFromSurface(s)
    # 미리보기
    pts = np.array([[s.PointAt(s.Domain(0).T0 + (s.Domain(0).T1 - s.Domain(0).T0) * a / 40,
                               s.Domain(1).T0 + (s.Domain(1).T1 - s.Domain(1).T0) * b / 16) for b in range(16)] for a in range(41)])
    V = np.array([[p.X, p.Y, p.Z] for row in pts for p in row])
    F = []
    for a in range(40):
        for b in range(16):
            i0 = a * 16 + b; i1 = (a + 1) * 16 + b; i2 = (a + 1) * 16 + (b + 1) % 16; i3 = a * 16 + (b + 1) % 16
            F += [[i0, i1, i2], [i0, i2, i3]]
    return brep, trimesh.Trimesh(V, np.array(F))


def main(src, dst):
    f = r.File3dm.Read(src)
    names = {i: l.FullPath for i, l in enumerate(f.Layers)}
    S = next((i for i, n in names.items() if n == 'S'), None)
    if S is None:
        L = r.Layer(); L.Name = 'S'; L.Color = (0, 0, 255, 255); S = f.Layers.Add(L)
    vis = [o for o in f.Objects if isinstance(o.Geometry, r.Brep) and o.Attributes.Visible
           and f.Layers[o.Attributes.LayerIndex].Visible and o.Attributes.LayerIndex != S]
    parts = [mesh_of(o.Geometry) for o in vis]
    model = trimesh.util.concatenate(parts)
    lo, hi = model.bounds
    prev, log, feet = [], [], []

    def add(b, tm, tag):
        a = r.ObjectAttributes(); a.LayerIndex = S; f.Objects.Add(b, a); prev.append(tm); log.append(tag)

    def hit(o, d, maxd=1e9):
        loc, _, _ = model.ray.intersects_location([o], [d])
        if not len(loc): return None
        dist = np.linalg.norm(loc - o, axis=1); k = np.argmin(dist)
        return loc[k] if dist[k] <= maxd else None

    PT = lo[2] - PLATE_GAP
    band_low = [m for m in parts if m.bounds[0][2] <= lo[2] + 0.01][0]
    R = max(-band_low.bounds[0][0], band_low.bounds[1][0])
    YB = max(-band_low.bounds[0][1], band_low.bounds[1][1])

    # 1) 바닥 기둥 4개 (곧게, 양끝 평평)
    for fx in (0.295, -0.295, 0.59, -0.59):
        x = fx * R; h = hit(np.array([x, 0, PT - 1]), UP)
        if h is None: continue
        b, tm = pipe([x, 0, PT - PLATE_T], h + UP * 0.15, 0.5, flat_start=True, flat_end=True); add(b, tm, '바닥 기둥 Ø1.0')
        feet.append((x, 0, 0.5))
    # 2) 가운데: 아래 몸통 Ø1.3 + 안쪽 몸통 Ø1.0 + 가지 4개 Ø0.96
    hb = hit(np.array([0, 0, PT - 1]), UP)
    b, tm = pipe([0, 0, PT - PLATE_T], hb + UP * 0.15, 0.65, flat_start=True, flat_end=True); add(b, tm, '아래 몸통 Ø1.3')
    feet.append((0, 0, 0.65))
    ib = hit(np.array([0, 0, 0.0]), -UP)
    b, tm = pipe(ib - UP * 0.15, [0, 0, -0.57], 0.5, flat_start=True, flat_end=True); add(b, tm, '안쪽 몸통 Ø1.0')
    fork = np.array([0, 0, -1.06])
    for deg in (40, 72):
        for sx in (1, -1):
            d = np.array([sx * math.cos(math.radians(deg)), 0, math.sin(math.radians(deg))])
            h = hit(np.array([0, 0, 0.0]), d)
            if h is None: continue
            b, tm = pipe(fork, h + d * 0.0, 0.48); add(b, tm, '안쪽 가지 Ø0.96')
    # 3) 옆: 가운데 기둥 Ø0.6 + 파베 발 갈고리 (아래 4줄 × 앞뒤)
    mm = model.copy(); mm.merge_vertices(); V, F = mm.vertices, mm.faces
    for sx in (1, -1):
        xc = sx * (R + 1.24)
        sel = np.all(sx * V[F][:, :, 0] > R - 0.83, axis=1) & np.all(V[F][:, :, 2] > lo[2] + (hi[2] - lo[2]) * 0.0 + 0.0, axis=1)
        sel &= np.all(V[F][:, :, 2] > band_low.bounds[1][2] + 0.02, axis=1) & np.all(V[F][:, :, 2] < 8, axis=1)
        teeth = []
        if sel.any():
            sub = mm.submesh([np.where(sel)[0]], append=True)
            for c in sub.split(only_watertight=False):
                a_, b_ = c.bounds
                if len(c.faces) >= 4: teeth.append((a_[2], (a_[1] + b_[1]) / 2, max(abs(a_[0]), abs(b_[0]))))
        rows = sorted(set(round(t[0], 2) for t in teeth))[:4]
        top = -1e9
        for zb, yc, xt in sorted(teeth):
            if round(zb, 2) not in rows: continue
            T = np.array([sx * (xt - 0.25), np.sign(yc) * 0.5, zb])
            Sx = np.array([xc, 0.0, zb - 1.8])
            ctrl = [Sx, Sx + UP * 1.1, T + np.array([sx * 0.9, 0, 0]), T + np.array([-sx * 0.12, 0, 0])]
            b, tm = curved_pipe(ctrl, 0.15); add(b, tm, '파베 갈고리 Ø0.3')
            top = max(top, Sx[2])
        print('  파베 발 줄(z):', rows)
        b, tm = pipe([xc, 0, PT - PLATE_T], [xc, 0, top + 0.36], 0.3, flat_start=True, flat_end=True); add(b, tm, '옆 기둥 Ø0.6')
        feet.append((xc, 0, 0.3))
    # 4) 헤드: 앞뒤 기둥 Ø1.0 + 곡선 6갈래 Ø0.55
    heads = [m for m in parts if m.bounds[0][2] > band_low.bounds[1][2] + 5]
    if heads:
        hm = trimesh.util.concatenate(heads); hz = hm.bounds[0][2]
        rail = max(heads, key=lambda m_: (m_.bounds[1][0] - m_.bounds[0][0]) * (m_.bounds[1][1] - m_.bounds[0][1]) / max(m_.bounds[1][2] - m_.bounds[0][2], 0.3))
        rlo, rhi = rail.bounds
        HY = max(-rlo[1], rhi[1]); HX = max(-rlo[0], rhi[0])
        for sy in (1, -1):
            yc = sy * max(0.67 * HY, YB + 0.9)
            ztop = hz - 5.0
            b, tm = pipe([0, yc, PT - PLATE_T], [0, yc, ztop + 0.7], 0.5, flat_start=True, flat_end=True); add(b, tm, '헤드 기둥 Ø1.0')
            feet.append((0, yc, 0.5))
            start = np.array([0, yc, ztop])
            targets = [(0, sy * (HY - 0.6)), (HX * 0.7, sy * HY * 0.84), (-HX * 0.7, sy * HY * 0.84),
                       (HX * 0.85, sy * HY * 0.44), (-HX * 0.85, sy * HY * 0.44), (0, sy * max(YB + 0.6, HY * 0.38))]
            for tx, ty in targets:
                h = hit(np.array([tx, ty, ztop]), UP)
                if h is None or h[2] < hz - 1.5: continue
                ctrl = [start, start + UP * 2.0, h - UP * 2.0, h + UP * 0.15]
                b, tm = curved_pipe(ctrl, 0.275, round_ends=False); add(b, tm, '헤드 갈래 Ø0.55')
    # 5) 바닥판 '+' 모양
    xs = [x for x, y, _ in feet]; ys = [y for x, y, _ in feet]
    shp = [box(min(xs) - 0.86, -1.5, max(xs) + 0.86, 1.5)]
    hy = [y for x, y, _ in feet if abs(y) > 1.0]
    if hy: shp.append(box(-1.02, min(hy) - 0.54 - 0.5, 1.02, max(hy) + 0.54 + 0.5))
    poly = unary_union(shp)
    crv = r.PolylineCurve([r.Point3d(x, y, PT - PLATE_T) for x, y in poly.exterior.coords])
    ext = r.Extrusion.Create(crv, PLATE_T, True)
    eb = ext.GetBoundingBox()
    if abs(eb.Max.Z - PT) > 1e-3: ext.Translate(r.Vector3d(0, 0, PT - eb.Max.Z))
    a = r.ObjectAttributes(); a.LayerIndex = S; f.Objects.Add(ext, a); log.append('바닥판 +')
    pl = trimesh.creation.extrude_polygon(poly, PLATE_T); pl.apply_translation((0, 0, PT - PLATE_T)); prev.append(pl)
    f.Write(dst, 7)
    print(Counter(log))
    d3s = Counter()
    for o in r.File3dm.Read(dst).Objects:
        if o.Attributes.LayerIndex == S and isinstance(o.Geometry, r.Brep): d3s[max_degree(o.Geometry)] += 1
    print('최대 차수별 개수:', dict(d3s))
    sc = trimesh.Scene(); info = []
    sc.add_geometry(model, node_name='m', geom_name='m'); info.append(dict(name='m', label='part', layer='기본값', vis=True, lc=[201, 201, 201], color=False))
    for k, tm in enumerate(prev):
        sc.add_geometry(tm, node_name=f's{k}', geom_name=f's{k}'); info.append(dict(name=f's{k}', label='support', layer='S', vis=True, lc=[40, 70, 230], color=True))
    base = dst[:-4]; sc.export(base + '.glb'); json.dump(info, open(base + '.json', 'w'), ensure_ascii=False)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
