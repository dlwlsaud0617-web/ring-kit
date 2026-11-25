#!/usr/bin/env python3
"""
support_ballring.py — 눕힌 볼(구슬) 반지 서포트 (회사 1125 파일 방식 그대로)

실측 규칙 (볼 Ø2.28 기준, 다른 크기는 볼 반지름 비율로 자동 조정)
  볼 1개마다
    가운데 기둥 : Ø0.52, 볼 중심 바로 아래, 바닥판 바닥(z=0)→볼 속 0.3 박힘, 위 끝 둥글게 / 아래 평평
    U자 받침   : Ø0.36 곡선 2갈래 (반지름 방향 안쪽·바깥쪽 대칭)
                 기둥 중간(볼 중심 -2.57)에서 출발 → 바깥으로 1.24 볼록 → 볼 아래 55° 지점에 닿음
                 곡선 차수 3, 양끝 둥글게
  바닥판 : 볼 반지 모양 링판 (안/밖 반지름 = 볼 링 반지름 ∓ 볼 반지름), 두께 0.3, 볼 바닥보다 2.86 아래
  표시 탭 : 3×3×0.3, 링 안쪽 앞(-Y), 호수 숫자 0.25 양각, 180° 돌려서
사용: python3 support_ballring.py 입력.3dm 출력.3dm [--label 17]
"""
import sys, math, json, argparse, numpy as np, rhino3dm as r, trimesh
from collections import Counter
sys.path.insert(0, '/home/user/ring-kit')
from pipe_rules import pipe, max_degree
from support_v3 import curved_pipe, mesh_of

RB0 = 1.14   # 기준 볼 반지름
# U자 받침 제어점 (반지름방향, 높이) — 기준 볼 실측값
FORK = [(0.86, -0.59), (0.94, -0.67), (1.08, -0.84), (1.21, -1.15), (1.24, -1.49), (1.17, -1.81),
        (1.02, -2.11), (0.78, -2.35), (0.49, -2.52), (0.27, -2.56), (0.16, -2.57)]
SEG = {  # 간단한 획 글꼴 (0~1 상자 안 가로/세로 획)
    '0': ['a', 'b', 'c', 'd', 'e', 'f'], '1': ['b', 'c'], '2': ['a', 'b', 'g', 'e', 'd'], '3': ['a', 'b', 'g', 'c', 'd'],
    '4': ['f', 'g', 'b', 'c'], '5': ['a', 'f', 'g', 'c', 'd'], '6': ['a', 'f', 'g', 'e', 'c', 'd'], '7': ['a', 'b', 'c'],
    '8': ['a', 'b', 'c', 'd', 'e', 'f', 'g'], '9': ['a', 'b', 'c', 'd', 'f', 'g'], '.': ['p']}


def seg_boxes(ch, x0, y0, w, h, t):
    S = {'a': (0, h - t, w, h), 'b': (w - t, h / 2, w, h), 'c': (w - t, 0, w, h / 2), 'd': (0, 0, w, t),
         'e': (0, 0, t, h / 2), 'f': (0, h / 2, t, h), 'g': (0, h / 2 - t / 2, w, h / 2 + t / 2), 'p': (w / 2 - t / 2, 0, w / 2 + t / 2, t)}
    return [(x0 + a, y0 + b, x0 + c, y0 + d) for a, b, c, d in (S[s] for s in SEG.get(ch, []))]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('src'); ap.add_argument('dst'); ap.add_argument('--label', default='')
    a = ap.parse_args()
    f = r.File3dm.Read(a.src)
    names = {i: l.FullPath for i, l in enumerate(f.Layers)}
    sup_layers = {i for i, n in names.items() if n.lower() == 's'}
    # 기존 서포트 지우고 처음부터
    for o in list(f.Objects):
        if o.Attributes.LayerIndex in sup_layers: f.Objects.Delete(o.Attributes.Id)
    S = next(iter(sup_layers)) if sup_layers else None
    if S is None:
        L = r.Layer(); L.Name = 's'; L.Color = (0, 0, 255, 255); S = f.Layers.Add(L)
    balls = []
    for o in f.Objects:
        g = o.Geometry
        if not isinstance(g, r.Brep) or o.Attributes.LayerIndex == S: continue
        m = mesh_of(g); lo, hi = m.bounds; e = hi - lo
        if e.max() - e.min() < 0.1 * e.max():          # 구슬(정육면 bbox)
            balls.append(((lo + hi) / 2, e.mean() / 2, m))
    C = np.array([b[0] for b in balls]); rb = float(np.mean([b[1] for b in balls]))
    A = np.c_[2 * C[:, 0], 2 * C[:, 1], np.ones(len(C))]
    sol = np.linalg.lstsq(A, C[:, 0] ** 2 + C[:, 1] ** 2, rcond=None)[0]
    cx, cy = sol[:2]; R = math.sqrt(sol[2] + cx ** 2 + cy ** 2)
    k = rb / RB0
    zc = float(C[:, 2].mean()); zb = zc - rb
    z0 = zb - 2.86 * k - 0.3                          # 바닥판 바닥
    prev, log = [], []

    def add(b, tm, tag):
        at = r.ObjectAttributes(); at.LayerIndex = S; f.Objects.Add(b, at); prev.append(tm); log.append(tag)

    for c, _, _ in balls:
        radial = np.array([c[0] - cx, c[1] - cy, 0]); radial /= np.linalg.norm(radial)
        # 가운데 기둥
        b, tm = pipe([c[0], c[1], z0], [c[0], c[1], zc - 1.08 * k], 0.26 * k, flat_start=True); add(b, tm, '가운데 기둥 Ø0.52')
        # U자 받침 2갈래
        for sgn in (1, -1):
            ctrl = [c + radial * sgn * u * k + np.array([0, 0, dz * k]) for u, dz in FORK]
            b, tm = curved_pipe(ctrl, 0.18 * k); add(b, tm, 'U자 받침 Ø0.36')
    # 바닥판: 링판
    ri, ro = R - rb - 0.015, R + rb + 0.04
    prof = r.PolylineCurve([r.Point3d(cx + x, cy, z) for x, z in [(ri, z0), (ro, z0), (ro, z0 + 0.3), (ri, z0 + 0.3), (ri, z0)]])
    rev = r.RevSurface.Create(prof, r.Line(r.Point3d(cx, cy, 0), r.Point3d(cx, cy, 1)), 0, 2 * math.pi)
    at = r.ObjectAttributes(); at.LayerIndex = S; f.Objects.Add(r.Brep.CreateFromRevSurface(rev, False, False), at); log.append('링 바닥판')
    ann = trimesh.creation.annulus(r_min=ri, r_max=ro, height=0.3, sections=96); ann.apply_translation((cx, cy, z0 + 0.15)); prev.append(ann)
    # 표시 탭 + 숫자 (앞쪽 -Y, 180° 회전)
    tc = np.array([cx, cy - (R - rb - 0.8)])
    bb = r.BoundingBox(r.Point3d(tc[0] - 1.5, tc[1] - 1.5, z0), r.Point3d(tc[0] + 1.5, tc[1] + 1.5, z0 + 0.3))
    add(r.Brep.CreateFromBoundingBox(bb), trimesh.creation.box(bounds=[[tc[0] - 1.5, tc[1] - 1.5, z0], [tc[0] + 1.5, tc[1] + 1.5, z0 + 0.3]]), '표시 탭')
    if a.label:
        w, h, t, gap = 0.9, 2.0, 0.25, 0.3
        tot = len(a.label) * w + (len(a.label) - 1) * gap
        x = -tot / 2
        for ch in a.label:
            for bx in seg_boxes(ch, x, -h / 2, w, h, t):
                # 180° 회전: (x,y) → (-x,-y)
                x0, y0, x1, y1 = -bx[2], -bx[3], -bx[0], -bx[1]
                lo_ = [tc[0] + x0, tc[1] + y0, z0 + 0.05]; hi_ = [tc[0] + x1, tc[1] + y1, z0 + 0.55]
                add(r.Brep.CreateFromBoundingBox(r.BoundingBox(r.Point3d(*lo_), r.Point3d(*hi_))), trimesh.creation.box(bounds=[lo_, hi_]), '숫자')
            x += w + gap
    f.Write(a.dst, 7)
    print(f'볼 {len(balls)}개, 볼 Ø{2*rb:.2f}, 링 반지름 {R:.3f}, 중심 ({cx:.2f},{cy:.2f})')
    print(Counter(log))
    d = Counter(max_degree(o.Geometry) for o in r.File3dm.Read(a.dst).Objects if o.Attributes.LayerIndex == S and isinstance(o.Geometry, r.Brep))
    print('최대 차수별:', dict(d))
    sc = trimesh.Scene(); info = []
    for i, (_, _, m) in enumerate(balls):
        sc.add_geometry(m, node_name=f'b{i}', geom_name=f'b{i}'); info.append(dict(name=f'b{i}', label='part', layer='기본값', vis=True, lc=[201, 201, 201], color=False))
    for i, tm in enumerate(prev):
        sc.add_geometry(tm, node_name=f's{i}', geom_name=f's{i}'); info.append(dict(name=f's{i}', label='support', layer='S', vis=True, lc=[40, 70, 230], color=True))
    base = a.dst[:-4]; sc.export(base + '.glb'); json.dump(info, open(base + '.json', 'w'), ensure_ascii=False)


if __name__ == '__main__':
    main()
