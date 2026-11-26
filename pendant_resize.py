#!/usr/bin/env python3
"""
pendant_resize.py — 펜던트 헤드를 다른 알 크기로 변형 (기출 변형용)

방법 (라이노 Scale 과 같음)
  * 알·커터·아래 링·위 레일·발 : 알 중심(바닥 높이) 기준 비율 = 새 알 / 기존 알 로 Scale
  * 목걸이 고리 : 크기는 그대로(체인 구멍 유지), 위치만 같은 비율로 이동
  * 알 위 표시 숫자 / '입고' 텍스트 : 새 알 크기로 교체
  * 기존 서포트(s 레이어)는 그대로 둠 → support_pendant.py 로 새로 생성
사용: python3 pendant_resize.py 입력.3dm 출력.3dm --stone 5.6   (서포트까지 자동, 빼려면 --no-support)
"""
import sys, re, math, argparse, numpy as np, rhino3dm as r
sys.path.insert(0, '/home/user/ring-kit')
from support_v3 import mesh_of
from support_ballring import seg_boxes


def rtf(text):
    out = []
    for ch in text:
        o = ord(ch)
        if ch == '\n': out.append(r'\par ')
        elif o < 128 and ch not in '\\{}': out.append(ch)
        else: out.append('\\u%d?' % (o - 65536 if o > 32767 else o))
    return r'{\rtf1\deff0{\fonttbl{\f0 Arial;}}\f0 \fs23{\f0 ' + ''.join(out) + r'\par}}'


def main():
    import trimesh
    override = {}
    ap = argparse.ArgumentParser(); ap.add_argument('src'); ap.add_argument('dst'); ap.add_argument('--stone', type=float, required=True); ap.add_argument('--no-support', dest='with_support', action='store_false')
    a = ap.parse_args()
    f = r.File3dm.Read(a.src)
    names = {i: l.FullPath for i, l in enumerate(f.Layers)}
    sup = {i for i, n in names.items() if n.lower() == 's'}
    objs = list(f.Objects)
    # 기존 알 크기: '5.35mm X 1' 텍스트 → 없으면 실측
    old = None
    for o in objs:
        if type(o.Geometry).__name__ == 'Text':
            mt = re.match(r'\s*([\d.]+)\s*mm', o.Geometry.PlainText)
            if mt and names[o.Attributes.LayerIndex].replace('.', '').isdigit(): old = float(mt.group(1))
    stone = [o for o in objs if isinstance(o.Geometry, r.Brep) and names[o.Attributes.LayerIndex].replace('.', '').isdigit()]
    sb = mesh_of(stone[0].Geometry).bounds
    if old is None: old = max(sb[1][0] - sb[0][0], sb[1][1] - sb[0][1])
    k = a.stone / old
    cx, cy = (sb[0][:2] + sb[1][:2]) / 2; stone_top = sb[1][2]
    metal = [o for o in objs if isinstance(o.Geometry, r.Brep) and o.Attributes.LayerIndex not in sup]
    zmin = min(mesh_of(o.Geometry).bounds[0][2] for o in metal)
    anchor = r.Point3d(cx, cy, zmin)
    T = r.Transform.Scale(anchor, k)
    glyph_z = None; glyph_layer = None; n_scaled = n_moved = 0
    for o in metal:
        m = mesh_of(o.Geometry); lo, hi = m.bounds; e = hi - lo; c = (lo + hi) / 2
        is_glyph = lo[2] > stone_top - 0.5 and not names[o.Attributes.LayerIndex].replace('.', '').isdigit()
        if is_glyph:
            glyph_z = (lo[2], hi[2]); glyph_layer = o.Attributes.LayerIndex
            f.Objects.Delete(o.Attributes.Id); continue
        off = math.hypot(c[0] - cx, c[1] - cy)
        is_loop = off > 1.5 and e[2] <= 2.5 and max(e) < 3.0
        g = o.Geometry.Duplicate(); mm = m.copy()
        if is_loop:
            d = np.array([(k - 1) * (c[0] - cx), (k - 1) * (c[1] - cy), (k - 1) * (c[2] - zmin)])
            g.Translate(r.Vector3d(*d)); mm.apply_translation(d); n_moved += 1
        else:
            g.Transform(T); n_scaled += 1
            mm.vertices = (mm.vertices - [cx, cy, zmin]) * k + [cx, cy, zmin]
        at = o.Attributes
        f.Objects.Delete(o.Attributes.Id); nid = f.Objects.Add(g, at); override[str(nid)] = mm
    # 텍스트 교체
    s_new = ('%g' % a.stone)
    for o in list(f.Objects):
        if type(o.Geometry).__name__ == 'Text' and re.match(r'\s*[\d.]+\s*mm', o.Geometry.PlainText):
            t = o.Geometry.Duplicate(); txt = re.sub(r'[\d.]+\s*mm', s_new + 'mm', o.Geometry.PlainText, count=1)
            t.SetRichText(rtf(txt), f.DimStyles[0]); at = o.Attributes
            f.Objects.Delete(o.Attributes.Id); f.Objects.Add(t, at)
    # 알 위 표시 숫자 새로
    if glyph_z is not None:
        z0, z1 = glyph_z[0] + (k - 1) * (glyph_z[0] - zmin), glyph_z[1] + (k - 1) * (glyph_z[0] - zmin)
        w, h, tk, gap = 0.55, 1.02, 0.15, 0.2
        tot = sum(0.25 if ch == '.' else w for ch in s_new) + gap * (len(s_new) - 1)
        x = cx - tot / 2
        for ch in s_new:
            ww = 0.25 if ch == '.' else w
            for bx in seg_boxes(ch, x, cy - h / 2, ww, h, tk):
                at = r.ObjectAttributes(); at.LayerIndex = glyph_layer
                nid = f.Objects.Add(r.Brep.CreateFromBoundingBox(r.BoundingBox(r.Point3d(bx[0], bx[1], z0), r.Point3d(bx[2], bx[3], z1))), at)
                override[str(nid)] = trimesh.creation.box(bounds=[[bx[0], bx[1], z0], [bx[2], bx[3], z1]])
            x += ww + gap
    tmp = a.dst[:-4] + '_tmp.3dm'; f.Write(tmp, 7)
    print(f'기존 알 {old} → 새 알 {a.stone} (비율 {k:.4f}), 확대 {n_scaled}개, 고리 이동 {n_moved}개')


    import os, support_pendant
    if a.with_support:
        support_pendant.main(tmp, a.dst, override); os.remove(tmp)
    else:
        os.replace(tmp, a.dst)


if __name__ == '__main__':
    main()
