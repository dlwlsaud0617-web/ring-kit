---
name: ring
description: 회사(갤젬 등) 라이노7 규격으로 반지 초안 .3dm + 4면 미리보기 생성. "/ring 8.5호 18wg 1캐럿 라운드 4발" 같은 한국어 주문 문장을 받으면 사용.
---

# /ring — 반지 초안 생성

작업 폴더: 이 저장소 루트(`ringgen.py`가 있는 곳). 규칙은 `STYLE_GUIDE.md` 참고.

## 1. 주문 문장 → 옵션
| 주문 표현 | 옵션 |
|---|---|
| 거래처 (기본 갤젬) | `--client` |
| `8.5호`, `#8.5` | `--size 8.5` |
| `18wg/18pg/18yg/14wg/pt950`, "화이트골드"=18wg, "로즈/핑크"=18pg, "백금"=pt950 | `--metal` |
| 라운드/프린세스(스퀘어)/오벌/쿠션/에메랄드 | `--shape round/princess/oval/cushion/emerald` |
| `1캐럿`, `1ct`, `5부`=0.5 | `--ct` |
| 실측 알 `7.4mm`, `8x6` (세로x가로) | `--stone` (ct와 같이 쓰면 ct는 라벨용) |
| `4발`, `6발` | `--prongs` |
| `파베 1.35mm 18개`, `사이드 1.3x20` | `--pave 1.35x18` |
| 밴드 `폭2.4 두께1.7` | `--band 2.4x1.7` (윗부분), `--band-bottom` (바닥) |
| 발 굵기 | `--prong-d` |
| 기타 메모 (예 `0.4`) | `--note` |

빠진 값: 소재 없으면 18wg, 모양 없으면 round, 알 정보가 전혀 없으면 사용자에게 물어본다. 호수는 필수.

## 2. 실행
```bash
python3 ringgen.py --client 갤젬 --size 8.5 --metal 18wg --shape round --ct 1 --prongs 4 --render
```
- 결과: `out/<거래처>_<모양>_<ct>_#<호수>.3dm` + `_preview.png` (가드링은 `YYMMDD_제품명_알mm_호수.3dm`)
- 처음이면 `pip install rhino3dm trimesh manifold3d numpy` 먼저.
- 미리보기 PNG를 Read로 보고 이상하면 (발이 밴드에서 떠 있음, 알이 밴드에 닿음 등) 옵션을 고쳐서 다시 실행.
- 결과 JSON의 내경, 알 크기, 예상 중량(g)을 사용자에게 표로 보여주고 3dm·PNG를 SendUserFile로 전달.

## 2-B. 가드링(이터니티) 주문 — `--style eternity`
"가드링", "이터니티", "알 둘러박기", "영근씨 가드링" 이 나오면 이 모드.

| 주문 표현 | 옵션 |
|---|---|
| `2.0mm`, `알 1.5` | `--stone 2.0` (필수) |
| 풀 / 3/4 / 하프 | `--coverage full / 3/4 / half` (기본 full) |
| 알 개수 지정 `28개` | `--count 28` (없으면 자동 계산) |
| `추가 4개` | `--spare 4` |
| `34개 입`, 입고 34 | `--received 34` |
| 제품명 `영근씨 가드링` | `--title 영근씨가드링` |
| 날짜 | `--date YYMMDD` (오늘 날짜로 기본 채움) |
| 밴드 직접 지정 | `--band 폭x두께` (기본: 폭 0.75×알+0.65, 두께 0.5×알+1.0) |

```bash
python3 ringgen.py --style eternity --client 갤젬 --title 영근씨가드링 --size 11.5 --metal 18wg --stone 2.0 --spare 4 --date 250701 --render
```
- 레이어는 B 시리즈 템플릿(`layers_eternity.json`): 알 → 크기 레이어(`2.0`), 커터 → `Rds_03`
- 결과 JSON의 `count`(필요 알 수), `need_with_spare`, `gap_mm`를 꼭 보고: 간격 < 0.1 이거나 > 0.3 이면 사용자에게 알리고 `--count`를 조정할지 물어보기.
- 안쪽 호수 양각, S 서포트는 라이노에서 추가.

## 3. 생성기가 못 하는 것 → 라이노에서 마무리하라고 안내
- 파베 시트 구멍: `커터` 레이어(숨김) 객체로 BooleanDifference
- `뒷판금까기`, 헤일로/클러스터, 시그넷, 카보숑 베젤, 하트 갤러리 같은 장식
- `S` 레이어 서포트(프린팅용), 렌더링
- 예상 중량은 커터 뺀 근사치(±10%)
