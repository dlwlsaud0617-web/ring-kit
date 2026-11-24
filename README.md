# ring-kit — 회사 규격 반지 3dm 자동 생성

- `STYLE_GUIDE.md` : 원본 5개 파일 분석 결과(규칙·치수·레이어·라벨)
- `ringgen.py` : 명령 한 줄 → 회사 레이어/좌표/라벨 규격의 .3dm 초안 + 4면 미리보기 (솔리테어 / 가드링 `--style eternity`)
- `.claude/skills/ring/SKILL.md` : Claude에서 `/ring 8.5호 18wg 1캐럿 라운드 4발` 처럼 쓰는 스킬
- `analysis/` : 원본 파일 렌더 이미지

```bash
pip install rhino3dm trimesh manifold3d numpy
python3 ringgen.py --size 8.5 --metal 18wg --shape round --ct 1 --pave 1.35x18 --render
```
미리보기는 Node + Playwright(Chromium) 필요.
