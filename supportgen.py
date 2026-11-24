#!/usr/bin/env python3
"""
supportgen.py — 회사 방식 3D프린팅 서포트 자동 생성 (S 레이어)

원본(에메랄드 6발 #10) 서포트 실측 규칙:
  * 바닥판 두께 0.3, 링 최저점보다 1.7mm 아래
  * 링: 바닥판→바깥면 수직 기둥 Ø0.6 (양쪽 6개씩, 바깥면에 0.3 박힘)
        가운데 받침 원뿔 Ø1.9 + 링 안쪽 몸통 Ø1.0 → 가지 Ø0.6 이 안쪽 윗면을 받침 (트리)
  * 그 외 부품(헤드 등): 아래를 향한 면(오버행)을 레이캐스트로 찾아 수직 기둥
사용:
  python3 supportgen.py 입력.3dm [출력.3dm]
"""
import sys, os, math
import numpy as np
import rhino3dm as r
import trimesh

PLATE_GAP, PLATE_T = 1.7, 0.3
PIN_D, TIP_D, TIP_L, EMBED = 0.6, 0.35, 0.6, 0.3
TRUNK_D, FOOT_D, BRANCH_D = 1.0, 1.9, 0.6
PIN_SPACING = 1.6


def P(v): return r.Point3d(*map(float, v))


def to_mesh(g):
    parts = []
    if isinstance(g, r.Brep):
        for f in g.Faces:
            m = f.GetMesh(r.MeshType.Any)
            if m: parts.append(m)
    elif isinstance(g, r.Mesh):
        parts.append(g)
    V, F, off = [], [], 0
    for m in parts:
        if len(m.Vertices) == 0: continue
        v = np.array([[p.X, p.Y, p.Z] for p in m.Vertices])
        for i in range(len(m.Faces)):
            f = m.Faces[i]
            F += [[f[0] + off, f[1] + off, f[2] + off]] + ([[f[0] + off, f[2] + off, f[3] + off]] if f[2] != f[3] else [])
        V.append(v); off += len(v)
    if not V: return None
    return trimesh.Trimesh(np.vstack(V), np.array(F), process=False)


class Out:
    def __init__(self, f, layer):
        self.f, self.layer, self.prev, self.n = f, layer, [], 0

    def _add(self, brep, tm):
        a = r.ObjectAttributes(); a.LayerIndex = self.layer
        self.f.Objects.Add(brep, a); self.prev.append(tm); self.n += 1

    def cyl(self, p0, p1, d):
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        ax = p1 - p0; h = float(np.linalg.norm(ax))
        if h < 0.05: return
        c = r.Circle(d / 2); c.Plane = r.Plane(P(p0), r.Vector3d(*map(float, ax / h)))
        self._add(r.Brep.CreateFromCylinder(r.Cylinder(c, h), True, True),
                  trimesh.creation.cylinder(radius=d / 2, segment=[p0, p1], sections=16))

    def cone(self, p0, p1, d0, d1):
        """p0쪽 지름 d0 → p1쪽 지름 d1 (잘린 원뿔) — Revolve로 생성"""
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        ax = p1 - p0; h = float(np.linalg.norm(ax)); u = ax / h
        tmp = np.array([1, 0, 0]) if abs(u[0]) < 0.9 else np.array([0, 1, 0])
        side = np.cross(u, tmp); side /= np.linalg.norm(side)
        pts = [p0, p0 + side * d0 / 2, p1 + side * d1 / 2, p1]
        crv = r.Polyline([P(q) for q in pts]).ToNurbsCurve() if hasattr(r.Polyline, 'ToNurbsCurve') else r.PolylineCurve([P(q) for q in pts])
        rev = r.RevSurface.Create(crv, r.Line(P(p0), P(p1)), 0, 2 * math.pi)
        self.f.Objects.Add(r.Brep.CreateFromRevSurface(rev, False, False), self._attr())
        self.n += 1
        # preview mesh
        n = 16; ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
        side2 = np.cross(u, side)
        ring = lambda c, rad: [c + rad * (math.cos(a) * side + math.sin(a) * side2) for a in ang]
        V = ring(p0, d0 / 2) + ring(p1, d1 / 2) + [p0, p1]
        F = []
        for k in range(n):
            k2 = (k + 1) % n
            F += [[k, k2, n + k2], [k, n + k2, n + k], [2 * n, k2, k], [2 * n + 1, n + k, n + k2]]
        self.prev.append(trimesh.Trimesh(np.array(V), np.array(F)))

    def _attr(self):
        a = r.ObjectAttributes(); a.LayerIndex = self.layer; return a

    def box(self, mn, mx):
        bb = r.BoundingBox(P(mn), P(mx))
        self._add(r.Brep.CreateFromBoundingBox(bb), trimesh.creation.box(bounds=[mn, mx]))


def pin(o, foot, top):
    """바닥판→접점: Ø0.6 기둥 + 끝 Ø0.35 원뿔"""
    foot, top = np.asarray(foot, float), np.asarray(top, float)
    d = top - foot; L = np.linalg.norm(d); u = d / L
    mid = top - u * TIP_L
    o.cyl(foot, mid, PIN_D)
    o.cone(mid, top, PIN_D, TIP_D)


def first_hit_up(mesh, x, y, z0):
    loc, ri, ti = mesh.ray.intersects_location([[x, y, z0]], [[0, 0, 1]], multiple_hits=True)
    if not len(loc): return None, None
    k = np.argmin(loc[:, 2])
    return loc[k], mesh.face_normals[ti[k]]


def ray_out(mesh, origin, direction):
    loc, ri, ti = mesh.ray.intersects_location([origin], [direction], multiple_hits=True)
    if not len(loc): return None
    d = np.linalg.norm(loc - origin, axis=1); return loc[np.argmin(d)]


def support_ring(o, m):
    """회사 링 서포트: 바닥판 + 바깥 기둥 12 + 가운데 트리"""
    lo, hi = m.bounds
    zb = lo[2]
    zp = zb - PLATE_GAP
    # 바깥 반지름(바닥) / 안쪽 반지름
    Ro = -zb
    Ri_hit = ray_out(m, np.array([0, 0, 0.0]), np.array([0, 0, -1.0]))
    Ri = -Ri_hit[2] if Ri_hit is not None else Ro - 2
    W = hi[1] - lo[1]
    o.box([lo[0] - 0.2, -1.5, zp - PLATE_T], [hi[0] + 0.2, 1.5, zp])
    # 바깥 기둥 (원본 비율: 바깥반지름 × 0.2~0.98)
    for fx in (0.20, 0.345, 0.495, 0.66, 0.815, 0.975):
        for s in (1, -1):
            x = s * fx * Ro
            hit, nrm = first_hit_up(m, x, 0.0, zp - 1)
            if hit is None: continue
            pin(o, [x, 0, zp], hit + np.array([0, 0, EMBED]))
    # 가운데 받침 + 몸통
    hit, _ = first_hit_up(m, 0, 0, zp - 1)
    o.cone([0, 0, zp], hit + np.array([0, 0, EMBED]), FOOT_D, FOOT_D * 0.7)
    z_tr0 = -Ri - 0.8                      # 링 바닥 안에서 시작
    z_tr1 = 0.5 * Ri
    o.cyl([0, 0, z_tr0], [0, 0, z_tr1], TRUNK_D)
    # 가지: 안쪽 윗면 40° / 62° / 80° 양쪽 (원본 위치)
    for deg, z0 in ((40, -0.1 * Ri), (62, -0.03 * Ri), (80, 0.2 * Ri)):
        for s in (1, -1):
            a = math.radians(deg)
            d = np.array([s * math.cos(a), 0, math.sin(a)])
            hit = ray_out(m, np.array([0, 0, 0.0]), d)
            if hit is None: continue
            pin(o, [0, 0, z0], hit + d * EMBED)
    return zp


def support_free(o, m, zp=None):
    """일반 부품: 아래 향한 면을 찾아 수직 기둥 (간격 1.6)"""
    lo, hi = m.bounds
    zp = lo[2] - PLATE_GAP if zp is None else zp
    pts = []
    xs = np.arange(lo[0] + 0.2, hi[0], 0.4); ys = np.arange(lo[1] + 0.2, hi[1], 0.4)
    for x in xs:
        for y in ys:
            hit, nrm = first_hit_up(m, x, y, zp - 1)
            if hit is None or nrm[2] > -0.35: continue
            pts.append(hit)
    pts = sorted(pts, key=lambda p: p[2])   # 낮은 곳 우선
    chosen = []
    for p in pts:
        if all(np.linalg.norm(p[:2] - q[:2]) >= PIN_SPACING for q in chosen):
            chosen.append(p)
    xy = np.array([c[:2] for c in chosen]) if chosen else np.array([[lo[0], lo[1]], [hi[0], hi[1]]])
    o.box([xy[:, 0].min() - 0.8, xy[:, 1].min() - 0.8, zp - PLATE_T], [xy[:, 0].max() + 0.8, xy[:, 1].max() + 0.8, zp])
    for c in chosen:
        pin(o, [c[0], c[1], zp], c + np.array([0, 0, EMBED]))
    return len(chosen)


def cluster(objs):
    """bbox 겹침으로 부품 묶기"""
    groups = []
    for i, (tm, bb) in enumerate(objs):
        hit = [g for g in groups if any(np.all(bb[0] <= b2[1] + 0.5) and np.all(b2[0] <= bb[1] + 0.5) for _, b2 in g)]
        new = [(tm, bb)]
        for g in hit:
            new += g; groups.remove(g)
        groups.append(new)
    return [trimesh.util.concatenate([t for t, _ in g]) for g in groups]


def main():
    src, dst = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(sys.argv[1])[0] + "_서포트.3dm")
    f = r.File3dm.Read(src)
    Sidx = next((i for i, L in enumerate(f.Layers) if L.FullPath == "S"), None)
    if Sidx is None:
        L = r.Layer(); L.Name = "S"; L.Color = (255, 191, 0, 255); Sidx = f.Layers.Add(L)
    objs = []
    for ob in f.Objects:
        a = ob.Attributes
        if a.LayerIndex == Sidx or not (f.Layers[a.LayerIndex].Visible and a.Visible): continue
        tm = to_mesh(ob.Geometry)
        if tm is not None and len(tm.faces): objs.append((tm, tm.bounds))
    parts = cluster(objs)
    o = Out(f, Sidx)
    report = []
    for m in parts:
        lo, hi = m.bounds; c = (lo + hi) / 2
        is_ring = abs(c[0]) < 2 and abs(c[2]) < 3 and (hi[0] - lo[0]) > 14
        if is_ring:
            support_ring(o, m); report.append(("링", o.n))
        else:
            n = support_free(o, m); report.append(("부품", n))
    f.Write(dst, 7)
    print("출력:", dst)
    for k, v in report: print(" ", k, v)
    print("서포트 객체 수:", o.n)
    # 미리보기용 glb/json (render.js 형식)
    base = os.path.splitext(dst)[0]
    scene = trimesh.Scene(); info = []
    for k, m in enumerate(parts):
        scene.add_geometry(m, node_name=f"p{k}", geom_name=f"p{k}")
        info.append(dict(name=f"p{k}", label="part", layer="기본값", vis=True, lc=[201, 201, 201], color=False))
    for k, m in enumerate(o.prev):
        scene.add_geometry(m, node_name=f"s{k}", geom_name=f"s{k}")
        info.append(dict(name=f"s{k}", label="support", layer="S", vis=True, lc=[230, 160, 20], color=True))
    scene.export(base + ".glb")
    import json; json.dump(info, open(base + ".json", "w"), ensure_ascii=False)


if __name__ == "__main__":
    main()
