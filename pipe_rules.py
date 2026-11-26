"""
pipe_rules.py — 회사 서포트 파이프 규칙대로 Brep 생성 (rhino3dm)

규칙 (STYLE_GUIDE C):
  * 단면 원형 (차수2 유리 원)
  * 끝: Round(반구). 바닥판에 서는 끝만 Flat
  * 차수 3 금지 → 단면/길이 방향 모두 차수 2 이하
구현: 프로파일(반원캡-직선-반원캡)을 파이프 축으로 Revolve → 닫힌 단일 솔리드
"""
import math
import numpy as np
import rhino3dm as r
import trimesh

W = math.sqrt(0.5)


def _P(v):
    return r.Point3d(*map(float, v))


def pipe(p0, p1, rad, flat_start=False, flat_end=False, rad_end=None):
    """p0→p1 원형 파이프 (rad_end 주면 끝 지름이 다른 테이퍼). 반환: (Brep, 미리보기 trimesh)"""
    r0 = rad; r1 = rad if rad_end is None else rad_end
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    ax = p1 - p0
    L = float(np.linalg.norm(ax))
    u = ax / L
    tmp = np.array([1.0, 0, 0]) if abs(u[0]) < 0.9 else np.array([0, 1.0, 0])
    s = np.cross(u, tmp); s /= np.linalg.norm(s)
    # (축방향 거리 t, 반지름방향 d, weight)
    pts = []
    if flat_start:
        pts += [(0, 0, 1), (0, r0 / 2, 1), (0, r0, 1)]
    else:
        pts += [(-r0, 0, 1), (-r0, r0, W), (0, r0, 1)]
    pts += [(L / 2, (r0 + r1) / 2, 1)]
    if flat_end:
        pts += [(L, r1, 1), (L, r1 / 2, 1), (L, 0, 1)]
    else:
        pts += [(L, r1, 1), (L + r1, r1, W), (L + r1, 0, 1)]
    crv = r.NurbsCurve(3, True, 3, 7)  # dim, rational, order(=degree2), count
    for i, (t, d, w) in enumerate(pts):
        q = p0 + u * t + s * d
        crv.Points[i] = r.Point4d(q[0] * w, q[1] * w, q[2] * w, w)
    for i, k in enumerate([0, 0, 1, 1, 2, 2, 3, 3]):
        crv.Knots[i] = k
    a0 = p0 - u * (0 if flat_start else r0)
    a1 = p1 + u * (0 if flat_end else r1)
    rev = r.RevSurface.Create(crv, r.Line(_P(a0), _P(a1)), 0, 2 * math.pi)
    brep = r.Brep.CreateFromRevSurface(rev, False, False)
    # 미리보기: 원기둥 + 반구
    if r0 == r1:
        parts = [trimesh.creation.cylinder(radius=r0, segment=[p0, p1], sections=20)]
    else:
        n = 20; v2 = np.cross(u, s); ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
        ring = lambda c, rr: [c + rr * (math.cos(t) * s + math.sin(t) * v2) for t in ang]
        V = ring(p0, r0) + ring(p1, r1) + [p0, p1]; F = []
        for k in range(n):
            k2 = (k + 1) % n; F += [[k, k2, n + k2], [k, n + k2, n + k], [2 * n, k2, k], [2 * n + 1, n + k, n + k2]]
        parts = [trimesh.Trimesh(np.array(V), np.array(F))]
    if not flat_start:
        sp = trimesh.creation.icosphere(subdivisions=2, radius=r0); sp.apply_translation(p0); parts.append(sp)
    if not flat_end:
        sp = trimesh.creation.icosphere(subdivisions=2, radius=r1); sp.apply_translation(p1); parts.append(sp)
    return brep, trimesh.util.concatenate(parts)


def max_degree(brep):
    d = 0
    for f in brep.Faces:
        ns = f.UnderlyingSurface().ToNurbsSurface()
        d = max(d, ns.Degree(0), ns.Degree(1))
    return d
