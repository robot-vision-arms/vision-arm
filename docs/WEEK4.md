# 4주차 — KPI · Pick & Place 통합 · 실측 정답·시연

문서 구성: **1. 현황 요약 → 2. 실행 방법 → 3~5. 팀원별 기록** (WEEK3.md 와 같은 틀).
팀원 1·2 절은 각자 채운다. 이 파일은 2026-11-20 팀원 3 이 처음 만들었다.

## 1. 현황 요약 (2026-11-20)

| 팀원 | 4주차 담당 (PROJECT.md 2.3) | 상태 | 남은 일 |
|---|---|---|---|
| 팀원 1 (동경) | KPI: `evaluate_kpi.py`, 100회 송신 테스트 + 모델 비교 | (팀원 1 작성) | |
| 팀원 2 (성민) | Pick & Place 한 사이클 통합 (`pick_place_node`) | (팀원 2 작성) | |
| 팀원 3 (가영) | 실측 정답 30+ 자세 수집, 시연 준비 | 🟡 임시 정답(화면 안내선 방식) 30장 수집·평가 (위치 MAE 1.85 mm, 각도 6.34°, 분류 100%). 인쇄 시트 PDF 준비 | 다음 주 인쇄 시트로 최종 정답 수집, 시연 준비 |

## 2. 실행 방법

### 실측 정답 수집 (팀원 3, `scripts/collect_ground_truth.py`)

```bash
# 자세 목록 (30개: 나사 24 = 0~165° 15° 간격 ×2, 너트 6, 위치는 모두 10 mm 격자점)
python3 scripts/collect_ground_truth.py --print-plan

# 최종용 인쇄 시트 (A4 가로, 100% 배율로 인쇄, 아래 100 mm 선을 자로 확인)
python3 scripts/collect_ground_truth.py --print-sheet data/ground_truth/pose_sheet.pdf

# 수집: print = 인쇄 시트 선 위에 놓음 (최종), overlay = 화면 안내선에 맞춤 (임시)
python3 scripts/collect_ground_truth.py --rotate 270 --method print
```

- 시트를 바꾸거나 움직이면 먼저 시트 사진을 다시 찍고 `compute_homography.py --aruco` 로 homography 를 새로 만든다.
- 화면: 노란 십자 = 목표 중심, 파란 선 = 나사 장축 방향, 노란 원 = 너트. `s` 저장, `n`/`p` 다음/이전, `q` 종료 (다시 실행하면 이어서).
- 시트 마커가 homography 예상 위치에서 3 px 넘게 벗어나면 화면에 `SHEET MOVED` 가 뜨고 저장이 막힌다.
- 일부만 다시 찍기: `--only gt002,gt013 --out-dir data/ground_truth/<새 폴더>` (기존 폴더를 덮어쓰지 않게 새 폴더로).
- **시트는 네 귀퉁이를 테이프로 고정한 뒤 시작한다.** 11-20 에는 고정 전 시트가 네 번 움직여 homography 를 매번 다시 만들었다.
- 예측값·오차는 수집 중에 보여 주지 않는다 (보면서 놓으면 정답이 예측 쪽으로 오염됨).
- 출력 `data/ground_truth/<method>_<날짜>/`: `poses.csv` (PROJECT.md 6.1 형식 `sample_id,image,x_mm,y_mm,angle_deg,part_type`), `images/`, 저장 순간 검출 결과 `preds.csv`.
- 폰이 세로로 보내는 동안은 `--rotate 270` (노드도 `-p camera_rotate_deg:=270`). `params.yaml` 은 0 그대로.

## 3. 팀원 1 (동경) 기록

## 4. 팀원 2 (성민) 기록

## 5. 팀원 3 (가영) 기록

### 2026-11-20 임시 실측 정답 30장 (화면 안내선 방식)

정답을 어떻게 만들지: 인쇄 시트(정답이 보정과 독립)가 최종, 오늘은 프린터 없이 화면 안내선(정답이 homography 에 의존)으로 임시 수집. 다음 주 인쇄 시트로 같은 자세 목록을 다시 수집해 최종 KPI 정답으로 쓴다.

| 항목 | 결과 |
|---|---|
| 수집 | 30/30 저장 (11:26~11:31, 약 6분). `data/ground_truth/overlay_20261120/` |
| 문제 ① 시트 이동 | 수집 직전 시트가 화면에서 약 120 px(≈45 mm) 왼쪽으로, 약간 돌아간 채 고정됨(테이프 붙이며 밀린 것으로 보임). 수집 30장 사이에는 마커 이동 0.5 px 이내로 안정. 안내선은 이동 전 homography 로 그려져 부품이 시트 격자가 아닌 **카메라 화면 기준 자리**에 놓임 |
| 처리 | 수집 사진(gt025)으로 homography 재계산 (한 점 빼고 잰 오차 0.237 mm, gt001 로 만든 것과 최대 0.22 mm 차). 정답 = 안내선 픽셀 위치를 새 homography 로 옮긴 값 (`poses.csv`, 원래 안내선 좌표는 `poses_guides.csv`). 그래서 이번 정답은 두 homography(각 약 0.2~0.4 mm)에 의존한다. 이 homography 를 현재 `data/calib/homography.npz` 로 적용 |
| 문제 ② 4개 다시 찍음 | gt002: 부품을 옮기기 전에 s 를 두 번 눌러 gt001 과 같은 장면. gt013·gt018·gt029: 시트 이동 탓에 목표가 오른쪽 마커 여백 안으로 들어가 부품이 마커와 붙어 검출 불가. → 11:49 `--only gt002,gt013,gt018,gt029` 로 `overlay_20261120_redo/` 에 다시 찍음. 그 전에 시트가 두 번 더 움직여(104 px, 147 px) 테이프로 고정 후 homography 재생성 (한 점 빼고 잰 오차 0.275 mm, `redo/homography_used.npz`). 4장 모두 저장 순간 마커 어긋남 1.2~1.4 px. 정답은 시트 좌표 그대로(격자점). 정답이 시트 자체 mm 좌표라 시트가 옮겨져도 사진마다 그때 homography 로 평가하면 합칠 수 있음 |
| 평가 (30개, `vision_node` 와 같은 처리·`params.yaml` 값) | 검출 30/30, **분류 30/30 (100%)**. **위치 MAE 1.85 mm** (중앙값 2.01, 최대 3.48, 기준 ≤ 2 mm). **각도 MAE 6.34°** (나사 24개, 중앙값 7.24, 최대 10.82, 기준 ≤ 3°). 다시 찍은 4개만 위치 MAE 1.49 mm, 처음 26개 1.90 mm. 합친 정답 `overlay_20261120/poses.csv` (다시 찍은 4개는 image 경로가 `../overlay_20261120_redo/images/`), 결과 `eval.csv`, 계산 `eval_gt.py` (같은 폴더) |
| 오차 해석 | 각도 오차는 부호 있는 평균 −0.40°, 표준편차 6.99° (30개 기준) → 치우침 없이 흩어짐. 같은 장면을 10초 연속 잰 검출 흔들림은 0.29°(WEEK2 11-20)라 검출 잡음보다 **화면 선에 손으로 맞춰 놓은 오차**가 크다고 봄. 위치는 평균 dx −1.27, dy +0.45 mm 치우침, 표준편차 약 1.1 mm. 이 정답으로는 KPI 달성 여부를 판단하지 않는다 |
| 개선 | `collect_ground_truth.py` 에 시트 이동 감지(마커 3 px 넘게 벗어나면 저장 막음) 추가. 오늘 사진 gt001 에 이동 전 homography 를 대면 132.5 px, 맞는 homography 는 1.0 px. 실제 스트림으로 저장 차단 확인 |
| 인쇄 시트 | `--print-sheet` 로 `data/ground_truth/pose_sheet.pdf` (+ PNG) 생성. PDF 297.0 × 210.0 mm. 마커·10 mm 격자 + 30개 자세(회색 170 선·번호) + 100 mm 확인선. 카메라 배율(2.6 px/mm)로 줄여 이진화하면 마커 4개만 덩어리로 남아 자세 선은 물체로 잡히지 않음, 마커 4개 검출 확인 |
| 참고 | 나사 검출: 11-20 오전 조명에서는 은색 나사가 이진화 후 205 px 로 `min_area_px`(500) 미만이라 미검출된 장면이 있었다. 수집 사진에서는 26/26 검출. 조명에 따라 흔들릴 수 있어 검출 담당과 공유 필요 |

#### 다음 주 (최종 정답)

1. `pose_sheet.pdf` 를 100% 배율로 인쇄, 100 mm 선 확인 → 지금 시트 자리에 테이프로 고정.
2. 시트 사진 → `compute_homography.py --aruco` → `homography.npz` 갱신.
3. `collect_ground_truth.py --rotate 270 --method print` 로 30장. 부품을 인쇄된 선·십자에 맞춤.
4. `evaluate_kpi.py`(팀원 1) 에 `poses.csv` 를 넘김.
