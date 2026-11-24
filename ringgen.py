#!/usr/bin/env python3
"""
ringgen.py — 회사(갤젬 등 거래처) 라이노7 작업 규격에 맞춘 반지 초안 .3dm 생성기

분석한 5개 원본 파일의 공통 규칙을 그대로 따른다:
  * 단위 mm, 허용오차 0.001, Rhino 7 포맷
  * 반지 정면 = XZ 평면(Front 뷰), 손가락 축 = Y, 링 중심 = 원점, 메인알은 +Z 위쪽
  * 내경 = (38 + 호수) / 3   (#8 → 15.33, #8.5 → 15.50, #12.5 → 16.83)
  * 레이어 템플릿: 기본값 / 알 / 발 / 레이어 01·02·04·05 / 뒷판금까기 / S / Dia / Dia::DD
  * Top 뷰(z=0)에 텍스트 라벨: 거래처명(좌상), 호수(좌), 소재(중앙 위), 알 정보(우)
  * 메인알 하단 ~0.9mm 클리어런스, 4발은 대각선(X) 배치, 파베 사이 '발' 비드 2열

사용 예:
  python3 ringgen.py --client 갤젬 --size 8.5 --metal 18wg --shape round --ct 1 --prongs 4 \
                     --pave 1.35x18 --render
"""
import argparse, math, os, sys, json
import numpy as np
import rhino3dm as r

HERE = os.path.dirname(os.path.abspath(__file__))

# ----------------------------------------------------------------- 규격 상수
LAYERS = [  # (이름, 색, 보임)  — 원본 파일과 동일한 이름/색
    ("기본값", (0, 0, 0), True),
    ("알", (255, 255, 255), True),
    ("발", (0, 127, 0), True),
    ("레이어 01", (255, 0, 0), True),
    ("레이어 02", (125, 38, 205), True),
    ("레이어 04", (0, 127, 0), True),
    ("레이어 05", (255, 255, 255), True),
    ("뒷판금까기", (255, 0, 0), True),
    ("S", (255, 191, 0), False),
    ("Dia", (255, 191, 191), True),
    ("Dia::DD", (255, 127, 0), True),
    ("커터", (255, 0, 0), False),   # 생성기 추가: 파베 시트 커터(라이노에서 BooleanDifference)
]
CUT_LAYERS = ("커터", "Rds_03")
METAL_LABEL = {"18wg": "18k WG", "18pg": "18k PG", "18yg": "18k YG", "14wg": "14k WG", "14pg": "14k PG",
               "14yg": "14k YG", "pt950": "PT950"}
DENSITY = {  # g/cm3
    "14k": 13.1, "14kw": 12.9, "14kp": 13.0, "14kg": 13.1, "14wg": 12.9, "14pg": 13.0, "14yg": 13.1,
    "18k": 15.5, "18wg": 15.6, "18pg": 15.1, "18yg": 15.5, "18kw": 15.6, "18kp": 15.1,
    "pt950": 20.7, "pt900": 21.5, "925": 10.4, "silver": 10.4,
}
CT_TO_MM = {"round": 6.5, "princess": 5.5, "oval": (7.7, 5.7), "cushion": 5.9, "emerald": (7.0, 5.0)}


def inner_diameter(size):
    return (38.0 + size) / 3.0


# ----------------------------------------------------------------- 유틸
def circle(pl, rad):
    c = r.Circle(float(rad)); c.Plane = pl
    return c


def P3(v):
    return r.Point3d(float(v[0]), float(v[1]), float(v[2]))


def V3(v):
    return r.Vector3d(float(v[0]), float(v[1]), float(v[2]))


def rtf(text):
    out = []
    for ch in text:
        o = ord(ch)
        if ch == "\n":
            out.append(r"\par ")
        elif o < 128 and ch not in "\\{}":
            out.append(ch)
        else:
            if o > 32767:
                o -= 65536
            out.append("\\u%d?" % o)
    return r"{\rtf1\deff0{\fonttbl{\f0 Arial;}}\f0 \fs23{\f0 " + "".join(out) + r"\par}}"


class Model:
    def __init__(self, layers=None):
        self.f = r.File3dm()
        self.f.Settings.ModelUnitSystem = r.UnitSystem.Millimeters
        self.f.Settings.ModelAbsoluteTolerance = 0.001
        self.layer = {}
        for name, col, vis in (layers or LAYERS):
            L = r.Layer()
            leaf = name.split("::")[-1]
            L.Name = leaf
            L.Color = (*col, 255)
            L.Visible = vis
            if "::" in name:
                L.ParentLayerId = self.f.Layers[self.layer[name.split("::")[0]]].Id
            self.layer[name] = self.f.Layers.Add(L)
        tpl = r.File3dm.Read(os.path.join(HERE, "text_template.3dm"))
        self.text_tpl = [o.Geometry for o in tpl.Objects][0]
        self.dim = self.f.DimStyles[0] if len(self.f.DimStyles) else tpl.DimStyles[0]
        self.tpl_dim = tpl.DimStyles[0]
        self.preview = []  # (name, layer, color, trimesh)

    def attr(self, layer, name=""):
        a = r.ObjectAttributes()
        a.LayerIndex = self.layer[layer]
        if name:
            a.Name = name
        return a

    def add(self, geom, layer, name=""):
        return self.f.Objects.Add(geom, self.attr(layer, name))

    def text(self, s, x, y, h):
        t = self.text_tpl.Duplicate()
        t.SetRichText(rtf(s), self.tpl_dim)
        try:
            t.SetTextHeight(self.tpl_dim, h)
        except Exception:
            pass
        cur = t.Plane.Origin
        t.Translate(V3((x - cur.X, y - cur.Y, -cur.Z)))
        self.f.Objects.Add(t, self.attr("기본값"))

    def prev(self, name, layer, color, mesh):
        self.preview.append((name, layer, color, mesh))


# ----------------------------------------------------------------- 밴드(샹크)
def bspline_closed_eval(cps, n=64):
    """uniform periodic cubic B-spline (2D) 평가 — 프로파일 실제 외곽 측정용"""
    cps = np.asarray(cps, float)
    m = len(cps)
    out = []
    for k in range(n):
        t = k / n * m
        i = int(math.floor(t)); u = t - i
        b = [(1 - u) ** 3 / 6, (3 * u**3 - 6 * u**2 + 4) / 6, (-3 * u**3 + 3 * u**2 + 3 * u + 1) / 6, u**3 / 6]
        p = sum(b[j] * cps[(i + j) % m] for j in range(4))
        out.append(p)
    return np.array(out)


def profile_cps(T, W):
    """콤포트핏 D자 단면 (dr: 내면→외면 0..T, dy: -W/2..W/2) — 외곽이 정확히 박스에 맞도록 보정"""
    base = np.array([
        (0.00, -0.30), (0.00, 0.30),            # 안쪽(손가락 닿는 면) 거의 평탄
        (0.10, 0.52), (0.62, 0.52),             # 옆면
        (1.00, 0.30), (1.00, -0.30),            # 위(바깥) 둥근 돔
        (0.62, -0.52), (0.10, -0.52),
    ])
    ev = bspline_closed_eval(base, 256)
    mn, mx = ev.min(0), ev.max(0)
    sc = np.array([T / (mx[0] - mn[0]), W / (mx[1] - mn[1])])
    return (base - mn) * sc + np.array([0, -W / 2])


def taper(phi, bottom, top):
    s = (1 + math.sin(phi)) / 2  # 0=바닥(6시) 1=윗면(12시)
    s = s ** 1.5
    return bottom + (top - bottom) * s


def build_shank(M, a):
    Rin = inner_diameter(a.size) / 2
    nu, order_v = 9, 4
    prof0 = profile_cps(1, 1)
    nv = len(prof0) + 3
    s = r.NurbsSurface.Create(3, True, 3, order_v, nu, nv)
    w45 = math.sqrt(0.5)
    for i in range(nu):
        phi = math.radians(-90 + 45 * i)
        T = taper(phi, a.thick_bottom, a.thick_top)
        W = taper(phi, a.width_bottom, a.width_top)
        prof = profile_cps(T, W)
        wt = 1.0 if i % 2 == 0 else w45
        k = 1.0 if i % 2 == 0 else 1 / w45
        for j in range(nv):
            dr, dy = prof[j % len(prof)]
            rho = (Rin + dr) * k
            x, z = rho * math.cos(phi), rho * math.sin(phi)
            s.Points[i, j] = r.Point4d(x * wt, dy * wt, z * wt, wt)
    for i, kv in enumerate([0, 0, 1, 1, 2, 2, 3, 3, 4, 4]):
        s.KnotsU[i] = kv
    for i in range(nv + order_v - 2):
        s.KnotsV[i] = i - 2
    M.add(r.Brep.CreateFromSurface(s), "기본값", "shank")
    # 미리보기 메shing (곡면 직접 샘플링)
    du, dv = s.Domain(0), s.Domain(1)
    NU, NV = 144, 40
    V = np.zeros((NU * NV, 3))
    for iu in range(NU):
        u = du.T0 + (du.T1 - du.T0) * iu / NU
        for iv in range(NV):
            v = dv.T0 + (dv.T1 - dv.T0) * iv / NV
            p = s.PointAt(u, v)
            V[iu * NV + iv] = (p.X, p.Y, p.Z)
    F = []
    for iu in range(NU):
        for iv in range(NV):
            a0 = iu * NV + iv; a1 = ((iu + 1) % NU) * NV + iv
            b0 = iu * NV + (iv + 1) % NV; b1 = ((iu + 1) % NU) * NV + (iv + 1) % NV
            F += [[a0, a1, b1], [a0, b1, b0]]
    import trimesh
    tm = trimesh.Trimesh(V, np.array(F), process=True)
    trimesh.repair.fix_normals(tm)
    M.prev("shank", "기본값", None, tm)
    return Rin, s


# ----------------------------------------------------------------- 보석(메시)
def stone_mesh(shape, L, Wd):
    """로컬좌표: 거들 중심 z=0, 테이블 +z. L=Y방향 길이, Wd=X방향 폭"""
    V, F = [], []
    if shape in ("round", "oval", "cushion"):
        n = 32
        rings = [(0.57 / 2, 0.165), (1 / 2, 0.015), (1 / 2, -0.015)]
        for rr, zz in rings:
            for k in range(n):
                t = 2 * math.pi * k / n
                V.append((rr * Wd * math.cos(t), rr * L * math.sin(t), zz * max(L, Wd)))
        depth = 0.445
    else:  # princess / emerald (사각)
        n = 4
        rings = [(0.75, 0.10), (1.0, 0.015), (1.0, -0.015)]
        for rr, zz in rings:
            for k in range(4):
                sx = [1, -1, -1, 1][k]; sy = [1, 1, -1, -1][k]
                V.append((sx * rr * Wd / 2, sy * rr * L / 2, zz * max(L, Wd)))
        depth = 0.58 if shape == "princess" else 0.45
    tc = len(V); V.append((0, 0, rings[0][1] * max(L, Wd)))
    cu = len(V); V.append((0, 0, -depth * max(L, Wd)))
    for k in range(n):
        k2 = (k + 1) % n
        F.append((tc, k, k2))
        for ring in range(2):
            a0, a1 = ring * n + k, ring * n + k2
            b0, b1 = (ring + 1) * n + k, (ring + 1) * n + k2
            F += [(a0, b0, b1), (a0, b1, a1)]
        F.append((2 * n + k, cu, 2 * n + k2))
    return np.array(V, float), np.array(F, int), depth * max(L, Wd), rings[0][1] * max(L, Wd)


def add_stone(M, shape, L, Wd, origin, xdir, ydir, zdir, layer, name):
    V, F, _, _ = stone_mesh(shape, L, Wd)
    R = np.column_stack([xdir, ydir, zdir])
    Vw = V @ R.T + np.asarray(origin)
    m = r.Mesh()
    for v in Vw:
        m.Vertices.Add(*map(float, v))
    for f in F:
        m.Faces.AddFace(int(f[0]), int(f[1]), int(f[2]))
    m.Normals.ComputeNormals(); m.Compact()
    M.add(m, layer, name)
    import trimesh
    M.prev(name, layer, (170, 215, 255), trimesh.Trimesh(Vw, F))


# ----------------------------------------------------------------- 금속 파츠
def add_cyl(M, p0, p1, rad, layer, name, prev_color=None):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    axis = p1 - p0; h = float(np.linalg.norm(axis))
    pl = r.Plane(P3(p0), V3(axis / h))
    cyl = r.Cylinder(circle(pl, rad), h)
    M.add(r.Brep.CreateFromCylinder(cyl, True, True), layer, name)
    import trimesh
    M.prev(name, layer, prev_color, trimesh.creation.cylinder(radius=rad, segment=[p0, p1], sections=24))
    return math.pi * rad * rad * h


def add_sphere(M, c, rad, layer, name, prev_color=None):
    M.add(r.Brep.CreateFromSphere(r.Sphere(P3(c), rad)), layer, name)
    import trimesh
    sp = trimesh.creation.icosphere(subdivisions=2, radius=rad); sp.apply_translation(c)
    M.prev(name, layer, prev_color, sp)
    return 4 / 3 * math.pi * rad**3


def add_torus(M, center_z, R, rr, layer, name):
    # 수평 링(갤러리 레일): XZ 단면 원을 Z축 회전
    pl = r.Plane(P3((R, 0, center_z)), V3((0, 1, 0)))
    crv = circle(pl, rr).ToNurbsCurve()
    axis = r.Line(P3((0, 0, 0)), P3((0, 0, 1)))
    rev = r.RevSurface.Create(crv, axis, 0, 2 * math.pi)
    M.add(r.Brep.CreateFromRevSurface(rev, False, False), layer, name)
    import trimesh
    t = trimesh.creation.torus(major_radius=R, minor_radius=rr, major_sections=64, minor_sections=16)
    t.apply_translation((0, 0, center_z))
    M.prev(name, layer, None, t)
    return 2 * math.pi**2 * R * rr * rr


# ----------------------------------------------------------------- 헤드(메인알 + 발)
def build_head(M, a, Rin):
    L, Wd = a.stone_L, a.stone_W
    D = max(L, Wd)
    _, _, pav, crown = stone_mesh(a.shape, L, Wd)
    top_out = Rin + a.thick_top
    zg = top_out + a.clearance + pav          # 거들 중심 높이
    add_stone(M, a.shape, L, Wd, (0, 0, zg), (1, 0, 0), (0, 1, 0), (0, 0, 1), "Dia::DD", "main_stone")
    pd = a.prong_d or max(0.8, min(1.3, 0.13 * D))
    vol = 0.0
    if a.shape in ("princess", "emerald"):
        angs = [45, 135, 225, 315]
        rx, ry = Wd / 2, L / 2
        pts_top = [(math.copysign(rx, math.cos(math.radians(t))), math.copysign(ry, math.sin(math.radians(t)))) for t in angs]
    else:
        n = a.prongs
        off = 45 if n == 4 else (0 if n == 6 else 90)
        angs = [off + 360 / n * k for k in range(n)]
        pts_top = [((Wd / 2) * math.cos(math.radians(t)), (L / 2) * math.sin(math.radians(t))) for t in angs]
    z_top = zg + crown + 0.35
    z_bot = Rin + a.thick_top * 0.45
    ymax = a.width_top / 2 - pd / 2 + 0.15
    rail_pts = []
    for k, (tx, ty) in enumerate(pts_top):
        d = math.hypot(tx, ty)
        ux, uy = tx / d, ty / d
        top = np.array([tx + ux * pd * 0.18, ty + uy * pd * 0.18, z_top])
        rb = max(0.3 * D, pd)
        bx, by = ux * rb, uy * rb
        if abs(by) > ymax:
            s_ = ymax / abs(by); bx *= s_; by *= s_
        # 밴드 곡면 위에서 시작하도록 z 보정
        bz = math.sqrt(max(0.0, (Rin + a.thick_top * 0.45) ** 2 - bx * bx))
        bot = np.array([bx, by, bz])
        vol += add_cyl(M, bot, top, pd / 2, "기본값", f"prong_{k+1}")
        vol += add_sphere(M, top, pd / 2 * 1.02, "기본값", f"prong_tip_{k+1}")
        rail_pts.append((bot, top))
    # 갤러리 레일 2줄 (원본: 발 기둥을 잇는 링 + 하단 링)
    for fz, nm in ((0.55, "gallery_upper"), (0.15, "gallery_lower")):
        zr = zg - pav * fz
        rads = []
        for bot, top in rail_pts:
            t = (zr - bot[2]) / (top[2] - bot[2])
            p = bot + (top - bot) * t
            rads.append(math.hypot(p[0], p[1]))
        if a.shape in ("princess", "emerald"):
            rads = [x * 0.85 for x in rads]
        Rr = float(np.mean(rads))
        if zr > top_out + 0.3:
            vol += add_torus(M, zr, Rr, pd * 0.42, "기본값", nm)
    return zg, pd, vol


# ----------------------------------------------------------------- 파베(사이드 스톤)
def build_pave(M, a, Rin, shank_srf):
    if not a.pave_n:
        return 0
    p, n_side = a.pave_d, a.pave_n // 2
    head_half = max(a.stone_L, a.stone_W) * 0.30 + 0.9
    gap = a.pave_gap
    cnt = 0
    for side in (1, -1):
        s_arc = head_half + p / 2
        for i in range(n_side):
            # 바깥 반지름 (테이퍼 반영) 기준 호 길이 → 각도
            phi = math.pi / 2
            for _ in range(3):
                Ro = Rin + taper(phi, a.thick_bottom, a.thick_top)
                phi = math.pi / 2 - side * s_arc / Ro
            Ro = Rin + taper(phi, a.thick_bottom, a.thick_top)
            Wl = taper(phi, a.width_bottom, a.width_top)
            nrm = np.array([math.cos(phi), 0, math.sin(phi)])
            tang = np.array([-math.sin(phi), 0, math.cos(phi)])
            c = nrm * (Ro - 0.25 * p)
            add_stone(M, "round", p, p, c, tang, (0, 1, 0), nrm, "Dia::DD", f"pave_{'R' if side>0 else 'L'}{i+1}")
            # 시트 커터 (라이노에서 BooleanDifference 용)
            add_cyl(M, c - nrm * (0.445 * p + 1.2), c + nrm * 0.4, p * 0.36, "커터", f"cut_hole_{cnt}")
            add_cyl(M, c - nrm * 0.02, c + nrm * 1.0, p / 2 + 0.02, "커터", f"cut_seat_{cnt}")
            cnt += 1
            if Wl < p + 0.6:
                print(f"  ! 경고: 파베 {i+1}번 위치 밴드폭 {Wl:.2f} < 알{p}+0.6 — 폭을 늘리세요", file=sys.stderr)
            s_arc += p + gap
    # 발(비드) : 각 틈마다 양쪽 2개, 원본 0.5mm 비드 패턴
    bead_r = a.bead_d / 2
    for side in (1, -1):
        s_arc = head_half
        for i in range(n_side + 1):
            phi = math.pi / 2
            for _ in range(3):
                Ro = Rin + taper(phi, a.thick_bottom, a.thick_top)
                phi = math.pi / 2 - side * s_arc / Ro
            Ro = Rin + taper(phi, a.thick_bottom, a.thick_top)
            nrm = np.array([math.cos(phi), 0, math.sin(phi)])
            for yy in (p * 0.385, -p * 0.385):
                add_sphere(M, nrm * (Ro + 0.02) + np.array([0, yy, 0]), bead_r, "발", f"bead", (90, 160, 90))
            s_arc += p + gap
    return cnt


# ----------------------------------------------------------------- 가드링(이터니티)
def eternity_defaults(a):
    D = a.eD
    a.gap = a.gap if a.gap else round(0.1 * D, 3)
    if not a.band:
        a.band = f"{0.75 * D + 0.65:.2f}x{0.5 * D + 1.0:.2f}"
    a.width_top, a.thick_top = map(float, a.band.lower().split("x"))
    a.width_bottom, a.thick_bottom = a.width_top, a.thick_top
    Rin = inner_diameter(a.size) / 2
    a.Rc = Rin + a.thick_top - a.seat_depth          # 알 거들 중심 반지름
    cover = {"full": 360, "3/4": 252, "half": 180, "1/2": 180}[a.coverage]  # 원본 #16: 21알×12°=252°
    pitch = (D + a.gap) / a.Rc                        # rad
    if cover == 360:
        a.eN = a.count or int(round(2 * math.pi / pitch))
        a.pitch = 2 * math.pi / a.eN
    else:
        a.eN = a.count or int(round(math.radians(cover) / pitch))
        a.pitch = pitch
    a.cover = cover


def build_eternity(M, a, Rin):
    D, N = a.eD, a.eN
    lay = f"{D:.1f}"
    if lay not in M.layer:
        lay = "Dia::DD" if "Dia::DD" in M.layer else "기본값"
    pd = a.prong_d or round(0.34 * D, 2)
    yp = D / 2 * 0.78                                  # 공유발 Y 위치(밴드 양끝)
    start = math.pi / 2 - a.pitch * (N - 1) / 2 if a.cover < 360 else math.pi / 2
    angs = [start + a.pitch * k for k in range(N)]
    for k, phi in enumerate(angs):
        nrm = np.array([math.cos(phi), 0, math.sin(phi)])
        tang = np.array([-math.sin(phi), 0, math.cos(phi)])
        c = nrm * a.Rc
        add_stone(M, "round", D, D, c, tang, (0, 1, 0), nrm, lay, f"stone_{k+1}")
        add_cyl(M, c - nrm * (0.445 * D + 1.5), c + nrm * 0.5, D * 0.33, "Rds_03", f"rds_hole_{k+1}")
        add_cyl(M, c - nrm * 0.03, c + nrm * 1.2, D / 2 + 0.02, "Rds_03", f"rds_seat_{k+1}")
    # 공유발: 알 사이(+양 끝) 밴드 양쪽 가장자리
    mids = [(angs[k] + angs[k + 1]) / 2 for k in range(N - 1)]
    if a.cover == 360:
        mids.append(angs[-1] + a.pitch / 2)
    else:
        mids = [angs[0] - a.pitch / 2] + mids + [angs[-1] + a.pitch / 2]
    Ro = Rin + a.thick_top
    for k, phi in enumerate(mids):
        nrm = np.array([math.cos(phi), 0, math.sin(phi)])
        for yy in (yp, -yp):
            b = nrm * (Ro - 0.4) + np.array([0, yy, 0])
            t = nrm * (a.Rc + 0.165 * D + 0.05) + np.array([0, yy, 0])
            add_cyl(M, b, t, pd / 2, "기본값", f"prong_{k+1}")
            add_sphere(M, t, pd / 2, "기본값", f"prong_tip_{k+1}")
    return pd


# ----------------------------------------------------------------- 미리보기 export
def export_preview(M, out_base):
    import trimesh
    scene = trimesh.Scene(); info = []
    for k, (name, layer, color, tm) in enumerate(M.preview):
        nm = f"o{k}"
        scene.add_geometry(tm, node_name=nm, geom_name=nm)
        info.append(dict(name=nm, label=name, layer=layer, vis=layer not in CUT_LAYERS,
                         lc=list(color) if color else [201, 201, 201], color=bool(color)))
    scene.export(out_base + ".glb")
    json.dump(info, open(out_base + ".json", "w"), ensure_ascii=False)


def apply_preview_cuts(M):
    """미리보기/중량용: 커터를 밴드에서 빼서 실제 시트 모양 재현 (3dm 원본은 커터 따로 보관)"""
    import trimesh
    cuts = [tm for n, l, c, tm in M.preview if l in CUT_LAYERS]
    if not cuts:
        return
    for k, (n, l, c, tm) in enumerate(M.preview):
        if n == "shank":
            try:
                res = trimesh.boolean.difference([tm] + cuts, engine="manifold")
                M.preview[k] = (n, l, c, res)
            except Exception as e:
                print("  (미리보기 불리언 실패:", e, ")", file=sys.stderr)


def metal_volume(M):
    v = 0
    for name, layer, color, tm in M.preview:
        if layer in ("기본값", "발") and tm.is_volume:
            v += abs(tm.volume)
    return v


# ----------------------------------------------------------------- main
def parse():
    ap = argparse.ArgumentParser(description="회사 규격 반지 초안 3dm 생성")
    ap.add_argument("--client", default="갤젬")
    ap.add_argument("--size", type=float, required=True, help="한국 호수 (#)")
    ap.add_argument("--metal", default="18wg", help="18wg/18pg/18yg/14wg/pt950 ...")
    ap.add_argument("--shape", default="round", choices=["round", "princess", "oval", "cushion", "emerald"])
    ap.add_argument("--ct", type=float, help="메인알 캐럿 (mm 미지정시 환산)")
    ap.add_argument("--stone", help="메인알 mm: '6.5' 또는 '7.7x5.7'(세로x가로)")
    ap.add_argument("--prongs", type=int, default=4, choices=[4, 6])
    ap.add_argument("--prong-d", type=float, default=0, help="발 굵기 mm (기본: 알크기×0.13, 0.8~1.3)")
    ap.add_argument("--pave", help="사이드 파베 '1.35x18' (알mm x 총개수)")
    ap.add_argument("--pave-gap", type=float, default=0.12)
    ap.add_argument("--bead-d", type=float, default=0.5, help="파베 발(비드) 지름")
    ap.add_argument("--band", default="", help="윗부분 밴드 폭x두께 mm")
    ap.add_argument("--band-bottom", default="", help="바닥 밴드 폭x두께 (기본: 위와 동일 폭, 두께+0.15)")
    ap.add_argument("--clearance", type=float, default=0.9, help="큘렛-밴드 간격")
    ap.add_argument("--note", default="", help="우측 추가 텍스트(예: 0.4)")
    ap.add_argument("--style", default="solitaire", choices=["solitaire", "eternity"],
                    help="solitaire=메인알 반지, eternity=가드링(알 둘러박기)")
    ap.add_argument("--coverage", default="full", choices=["full", "3/4", "half", "1/2"], help="가드링 알 배치 범위")
    ap.add_argument("--count", type=int, default=0, help="가드링 알 개수 강제(기본 자동)")
    ap.add_argument("--gap", type=float, default=0, help="가드링 알 간격(기본 알×0.1)")
    ap.add_argument("--seat-depth", type=float, default=0.3, help="밴드 외면→알 거들 깊이")
    ap.add_argument("--spare", type=int, default=0, help="추가(여분) 알 개수 라벨")
    ap.add_argument("--received", type=int, default=0, help="입고 알 개수 라벨 (예: 34개 입)")
    ap.add_argument("--title", default="", help="제품명 라벨 (예: 영근씨 가드링)")
    ap.add_argument("--date", default="", help="파일명 앞 날짜 YYMMDD")
    ap.add_argument("--out", default="")
    ap.add_argument("--render", action="store_true")
    a = ap.parse_args()
    if a.style == "eternity":
        if not a.stone:
            ap.error("가드링은 --stone 알지름(mm) 필수")
        a.eD = float(a.stone)
        eternity_defaults(a)
        if not a.out:
            date = (a.date + "_") if a.date else ""
            title = a.title or "가드링"
            a.out = os.path.join("out", f"{date}{title}_{a.eD:g}mm_{a.size:g}.3dm".replace(" ", ""))
        return a
    if not a.band:
        a.band = "2.2x1.6"
    # 알 크기
    if a.stone:
        if "x" in a.stone:
            L, W = map(float, a.stone.lower().split("x"))
        else:
            L = W = float(a.stone)
    else:
        ct = a.ct or 1.0
        base = CT_TO_MM[a.shape]
        k = ct ** (1 / 3)
        L, W = (base[0] * k, base[1] * k) if isinstance(base, tuple) else (base * k, base * k)
    a.stone_L, a.stone_W = round(L, 2), round(W, 2)
    a.width_top, a.thick_top = map(float, a.band.lower().split("x"))
    if a.band_bottom:
        a.width_bottom, a.thick_bottom = map(float, a.band_bottom.lower().split("x"))
    else:
        a.width_bottom, a.thick_bottom = a.width_top, a.thick_top + 0.15
    if a.pave:
        d, n = a.pave.lower().split("x"); a.pave_d, a.pave_n = float(d), int(n)
        need = a.pave_d + 0.6
        if a.width_top < need:
            print(f"  * 밴드 윗폭 {a.width_top} → 파베용 {need:.2f}로 자동 확장", file=sys.stderr)
            a.width_top = round(need, 2)
            a.width_bottom = max(a.width_bottom, a.width_top)
    else:
        a.pave_d, a.pave_n = 0, 0
    size_s = ("%g" % a.size)
    if not a.out:
        ct_s = (f"{a.ct:g}ct" if a.ct else f"{a.stone_L:g}mm")
        a.out = os.path.join("out", f"{a.client}_{a.shape}_{ct_s}_#{size_s}.3dm")
    return a


def main_eternity(a):
    layers = json.load(open(os.path.join(HERE, "layers_eternity.json"), encoding="utf-8"))
    M = Model(layers)
    Rin, srf = build_shank(M, a)
    pd = build_eternity(M, a, Rin)
    size_s = "%g" % a.size
    metal = METAL_LABEL.get(a.metal.lower(), a.metal)
    # 라벨 (가드링 원본 배치: 거래처 / 제품명+소재+호수 / 알 수량)
    M.text(a.client, -24.0, 9.0, 3.0)
    M.text(f"{a.title or '가드링'}\n{metal} #{size_s}", -24.0, 3.0, 2.0)
    lbl = f"{a.eD:g}mm X {a.eN}" + (f" ({a.received}개 입)" if a.received else "")
    M.text(lbl, 14.0, 4.0, 1.5)
    if a.spare:
        M.text(f"추가 {a.eD:g}mm X {a.spare}", 14.0, 1.5, 1.5)
    if a.note:
        M.text(a.note, 14.0, -1.0, 1.5)
    M.f.Write(a.out, 7)
    apply_preview_cuts(M)
    vol = metal_volume(M)
    dens = DENSITY.get(a.metal.lower(), 15.5)
    gap_real = a.pitch * a.Rc - a.eD
    summary = dict(
        file=a.out, client=a.client, title=a.title, size=f"#{size_s}", inner_d=round(inner_diameter(a.size), 2),
        metal=metal, coverage=a.coverage, stone=f"{a.eD:g}mm", count=a.eN, need_with_spare=a.eN + a.spare,
        gap_mm=round(gap_real, 3), band=f"{a.width_top}x{a.thick_top}", prong_d=pd,
        stone_center_R=round(a.Rc, 2), metal_volume_mm3=round(vol, 1), est_weight_g=round(vol / 1000 * dens, 2),
    )
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    return M


def main():
    a = parse()
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    if a.style == "eternity":
        M = main_eternity(a)
        finish_render(M, a)
        return
    M = Model()
    Rin, srf = build_shank(M, a)
    zg, pd, _ = build_head(M, a, Rin)
    npave = build_pave(M, a, Rin, srf)
    # Top 뷰 라벨 (원본 배치 규칙)
    size_s = "%g" % a.size
    M.text(a.client, -23.6, 5.0, 3.5)
    M.text(f"#{size_s}", -22.0, -2.0, 3.5)
    M.text(a.metal, -6.0, 11.5, 3.5)
    if a.ct:
        stone_lbl = f"{a.ct:g}ct"
    elif a.stone_L == a.stone_W:
        stone_lbl = f"{a.stone_L:g}mm"
    else:
        stone_lbl = f"{a.stone_L:g}x{a.stone_W:g}"
    M.text(stone_lbl, 17.0, 3.0, 2.5)
    if a.pave_n:
        M.text(f"{a.pave_d:g}mmx{a.pave_n}", 17.0, -1.0, 1.5)
    if a.note:
        M.text(a.note, 17.0, -4.0, 1.5)
    M.f.Write(a.out, 7)
    apply_preview_cuts(M)
    vol = metal_volume(M)
    dens = DENSITY.get(a.metal.lower(), 15.5)
    summary = dict(
        file=a.out, client=a.client, size=f"#{size_s}", inner_d=round(inner_diameter(a.size), 2),
        metal=a.metal, shape=a.shape, stone=f"{a.stone_L}x{a.stone_W}mm", ct=a.ct,
        prongs=a.prongs, prong_d=round(pd, 2), band_top=f"{a.width_top}x{a.thick_top}",
        band_bottom=f"{a.width_bottom}x{a.thick_bottom}", girdle_z=round(zg, 2),
        pave=f"{a.pave_d}mm x {npave}" if npave else None,
        metal_volume_mm3=round(vol, 1), est_weight_g=round(vol / 1000 * dens, 2),
    )
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    finish_render(M, a)


def finish_render(M, a):
    if a.render:
        base = os.path.splitext(a.out)[0]
        export_preview(M, base)
        import subprocess
        subprocess.run(["node", os.path.join(HERE, "render.js"), base, a.metal], check=False)
        for ext in (".glb", ".json"):
            try: os.remove(base + ext)
            except OSError: pass


if __name__ == "__main__":
    main()
