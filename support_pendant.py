#!/usr/bin/env python3
"""
support_pendant.py — 펜던트 헤드(목걸이 4발 등) 서포트 (회사 261125 샤론 5부 4발 목걸이 방식)

원리
  1. 서포트는 발(prong) 사이 방향(발 각도의 중간)에만 → 발은 안 건드림
  2. 아래 링  : 안쪽 기둥 Ø0.72 (링 관 중심보다 0.15 안쪽), 링 밑면에 0.4 박힘
  3. 위 레일  : 바깥 기둥 Ø0.72 (레일 바깥 + 0.48) 이 아래 링 높이까지 올라오고
               → 갈고리 곡선 Ø0.36 이 안쪽 위로 휘어 레일 밑면(관 중심 +0.08)에 닿음
  4. 목걸이 고리: 고리 바로 아래 기둥 Ø0.72, 고리에 0.36 박힘
  5. 바닥판   : 원(가장 바깥 기둥 + 0.66) + 고리 기둥용 귀, 두께 0.3, 모델 최저점 2.75 아래
  6. 기둥 = 곧게(차수1), 위 끝 둥글게 / 갈고리 = 곡선(차수3), 양끝 둥글게
사용: python3 support_pendant.py 입력.3dm 출력.3dm
"""
import sys, math, json, numpy as np, rhino3dm as r, trimesh
from collections import Counter
from shapely.geometry import Point, box
from shapely.ops import unary_union
from shapely import affinity
sys.path.insert(0, '/home/user/ring-kit')
from pipe_rules import pipe, max_degree
from support_v3 import curved_pipe, mesh_of

POST_R, HOOK_R, GAP, PT_T = 0.36, 0.18, 2.75, 0.3
# 기준 갈고리 (원본) : 접점 C0 → 기둥 끝 E0, 반지름방향 r / 높이 z
HOOK0 = [(2.183, 1.562), (2.315, 1.484), (2.561, 1.287), (2.837, 0.899), (2.994, 0.45), (3.012, 0.135), (3.0, -0.017)]


def main(src, dst):
    f = r.File3dm.Read(src)
    names = {i: l.FullPath for i, l in enumerate(f.Layers)}
    sup = {i for i, n in names.items() if n.lower() == 's'}
    for o in list(f.Objects):
        if o.Attributes.LayerIndex in sup: f.Objects.Delete(o.Attributes.Id)
    S = next(iter(sup)) if sup else None
    if S is None:
        L = r.Layer(); L.Name = 's'; L.Color = (3, 0, 255, 255); S = f.Layers.Add(L)
    skip = {i for i, n in names.items() if n in ('Rds_03',) or n.replace('.', '').isdigit()}
    stone = [o for o in f.Objects if isinstance(o.Geometry, r.Brep) and names[o.Attributes.LayerIndex].replace('.', '').isdigit()]
    parts = []
    for o in f.Objects:
        if not isinstance(o.Geometry, r.Brep) or o.Attributes.LayerIndex in skip | {S}: continue
        parts.append(mesh_of(o.Geometry))
    if stone:
        sb = mesh_of(stone[0].Geometry).bounds; cx, cy = (sb[0][:2] + sb[1][:2]) / 2; stone_top = sb[1][2]
    else:
        allb = trimesh.util.concatenate(parts).bounds; cx, cy = (allb[0][:2] + allb[1][:2]) / 2; stone_top = 1e9
    parts = [m for m in parts if m.bounds[0][2] < stone_top - 0.5]      # 알 위 표시 글자 제외
    model = trimesh.util.concatenate(parts)
    lo, hi = model.bounds
    PT = lo[2] - GAP

    def rad_of(m):
        v = m.vertices; return np.hypot(v[:, 0] - cx, v[:, 1] - cy)

    rings, prongs, loops = [], [], []
    for m in parts:
        a, b = m.bounds; e = b - a; c = (a + b) / 2
        off = math.hypot(c[0] - cx, c[1] - cy)
        if off < 0.3 and e[2] < 1.4 and min(e[0], e[1]) > 2.5: rings.append(m)
        elif e[2] > 2.5: prongs.append(m)
        elif off > 1.5: loops.append(m)
    rings.sort(key=lambda m: m.bounds[0][2])
    pang = sorted(math.atan2(((m.bounds[0] + m.bounds[1]) / 2)[1] - cy, ((m.bounds[0] + m.bounds[1]) / 2)[0] - cx) for m in prongs)
    mids = [(pang[i] + (pang[(i + 1) % len(pang)] + (2 * math.pi if i == len(pang) - 1 else 0))) / 2 for i in range(len(pang))]
    prev, log, feet = [], [], []

    def add(b, tm, tag):
        a_ = r.ObjectAttributes(); a_.LayerIndex = S; f.Objects.Add(b, a_); prev.append(tm); log.append(tag)

    def hit(x, y):
        loc, _, _ = model.ray.intersects_location([[x, y, PT - 1]], [[0, 0, 1.0]])
        return None if not len(loc) else loc[np.argmin(loc[:, 2])]

    low = rings[0]; rl = rad_of(low)
    r_in_post = (rl.min() + rl.max()) / 2 - 0.15
    low_mid_z = (low.bounds[0][2] + low.bounds[1][2]) / 2
    for a in mids:   # 아래 링 안쪽 기둥
        x, y = cx + r_in_post * math.cos(a), cy + r_in_post * math.sin(a)
        h = hit(x, y)
        if h is None: continue
        b, tm = pipe([x, y, PT - 0.2], [x, y, h[2] + 0.4 - POST_R], POST_R, flat_start=True); add(b, tm, '아래 링 기둥 Ø0.72')
        feet.append((x, y))
    if len(rings) > 1:   # 위 레일: 바깥 기둥 + 갈고리
        up = rings[1]; ru = rad_of(up)
        r_out = ru.max() + 0.48
        C = ((ru.min() + ru.max()) / 2 + 0.08, up.bounds[0][2] + 0.17)
        E = (r_out, low_mid_z - 0.02)
        C0, E0 = HOOK0[0], HOOK0[-1]
        sr = (E[0] - C[0]) / (E0[0] - C0[0]); sz = (C[1] - E[1]) / (C0[1] - E0[1])
        for a in mids:
            ca, sa = math.cos(a), math.sin(a)
            x, y = cx + r_out * ca, cy + r_out * sa
            b, tm = pipe([x, y, PT - 0.2], [x, y, E[1]], POST_R, flat_start=True); add(b, tm, '레일 바깥 기둥 Ø0.72')
            feet.append((x, y))
            ctrl = []
            for rr_, zz in HOOK0:
                rn = C[0] + (rr_ - C0[0]) * sr; zn = E[1] + (zz - E0[1]) * sz
                ctrl.append([cx + rn * ca, cy + rn * sa, zn])
            b, tm = curved_pipe(ctrl, HOOK_R); add(b, tm, '갈고리 Ø0.36')
    for m in loops:      # 목걸이 고리
        a, b_ = m.bounds
        lowv = m.vertices[m.vertices[:, 2] < a[2] + 0.08]; x, y = lowv[:, 0].mean(), lowv[:, 1].mean()   # 고리 맨 아래점
        b, tm = pipe([x, y, PT - 0.2], [x, y, a[2] + 0.36 - POST_R + POST_R], POST_R, flat_start=True); add(b, tm, '고리 기둥 Ø0.72')
        feet.append((x, y))
    # 바닥판: 원 + 귀
    nloop = len(loops)
    rmax = max(math.hypot(x - cx, y - cy) for x, y in feet[:len(feet) - nloop])
    shp = [Point(cx, cy).buffer(rmax + POST_R + 0.3, 64)]
    for (lx, ly) in feet[len(feet) - nloop:]:
        ang = math.degrees(math.atan2(ly - cy, lx - cx)); d = math.hypot(lx - cx, ly - cy)
        ear = box(d - 0.9, -0.36, d + POST_R + 0.3, 0.36)
        ear = affinity.rotate(ear, ang, origin=(0, 0)); ear = affinity.translate(ear, cx, cy)
        shp.append(ear)
    poly = unary_union(shp).simplify(0.01)
    crv = r.PolylineCurve([r.Point3d(x, y, PT - PT_T) for x, y in poly.exterior.coords])
    ext = r.Extrusion.Create(crv, PT_T, True)
    eb = ext.GetBoundingBox()
    if abs(eb.Max.Z - PT) > 1e-3: ext.Translate(r.Vector3d(0, 0, PT - eb.Max.Z))
    a_ = r.ObjectAttributes(); a_.LayerIndex = S; f.Objects.Add(ext, a_); log.append('바닥판')
    pl = trimesh.creation.extrude_polygon(poly, PT_T); pl.apply_translation((0, 0, PT - PT_T)); prev.append(pl)
    f.Write(dst, 7)
    print(f'발 {len(prongs)}개, 링 {len(rings)}개, 고리 {len(loops)}개, 서포트 방향(도): {[round(math.degrees(a)) for a in mids]}')
    print(Counter(log))
    print('최대 차수별:', dict(Counter(max_degree(o.Geometry) for o in r.File3dm.Read(dst).Objects if o.Attributes.LayerIndex == S and isinstance(o.Geometry, r.Brep))))
    sc = trimesh.Scene(); info = []
    sc.add_geometry(model, node_name='m', geom_name='m'); info.append(dict(name='m', label='part', layer='기본값', vis=True, lc=[201, 201, 201], color=False))
    for i, tm in enumerate(prev):
        sc.add_geometry(tm, node_name=f's{i}', geom_name=f's{i}'); info.append(dict(name=f's{i}', label='support', layer='S', vis=True, lc=[40, 70, 230], color=True))
    base = dst[:-4]; sc.export(base + '.glb'); json.dump(info, open(base + '.json', 'w'), ensure_ascii=False)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
