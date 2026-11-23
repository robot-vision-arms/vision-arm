# 4주차 — KPI · Pick & Place 통합 · 실측 정답·시연

문서 구성: **1. 현황 요약 → 2. 실행 방법 → 3~5. 팀원별 기록** (WEEK3.md 와 같은 틀).
팀원 1·2 절은 각자 채운다. 이 파일은 2026-11-20 팀원 3 이 처음 만들었다.

## 1. 현황 요약 (2026-11-20)

| 팀원 | 4주차 담당 (PROJECT.md 2.3) | 상태 | 남은 일 |
|---|---|---|---|
| 팀원 1 (동경) | KPI: `evaluate_kpi.py`, 100회 송신 테스트 + 모델 비교 | 🟡 `evaluate_kpi.py` 완성 (11-20). 임시 정답 30장으로 가영 님 결과 재현 (위치 1.86 mm, 각도 6.34°) | 인쇄 시트 정답으로 최종 KPI, 실물 폰 입력 100회 송신·P95, 초록 시트 PDF, 자체 사진 모델 재평가 |
| 팀원 2 (성민) | Pick & Place 한 사이클 통합 (`pick_place_node`) | (팀원 2 작성) | |
| 팀원 3 (가영) | 실측 정답 30+ 자세 수집, 시연 준비 | 🟡 임시 정답(화면 안내선 방식) 30장 수집·평가 (위치 MAE 1.85 mm, 각도 6.34°, 분류 100%). 인쇄 시트 PDF 준비. **최종 정답은 초록 시트로 결정** (11-23), `pose_sheet_green.pdf` 생성 | 초록 시트 인쇄·채도 확인 후 최종 정답 수집, 시연 준비 |

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

### 2026-11-20 KPI 계산 `scripts/evaluate_kpi.py` (계산은 `vision_pick_place/kpi.py`)

```bash
# 정답 사진으로 위치·각도·분류 (vision_node 와 같은 처리, 시차 보정 끈 값·켠 값 둘 다) + 송신 + Gazebo 집기
PYTHONPATH=src/vision_pick_place python scripts/evaluate_kpi.py \
    --truth data/ground_truth/<폴더>/poses.csv --per-image-homography \
    --comm logs/comm_<날짜>_*.csv --pick "logs/sim_pick_*.csv" --out logs/kpi_<날짜>.json
```

- 위치·각도: 사진마다 같은 처리로 검출 → 정답 위치에서 15 mm 안의 가장 가까운 검출과 대응 (없으면 미검출, 분류에서는 틀린 것으로 셈).
  `--per-image-homography` 는 사진마다 시트 마커로 호모그래피를 다시 구한다 (촬영 중 시트가 움직였을 때). `--preds` 로 저장된 검출을 쓸 수도 있다.
- 송신: `logs/comm_*.csv` 의 처음 100회 시도 중 ACK 비율, ACK 받은 것 중 시계 튐 없는 지연의 P95.
- Pick: `logs/sim_pick_*.csv` 의 물체별 결과. 결과마다 기준 대비 달성/미달과 측정 장비를 같이 출력.
- 테스트 8개 (`test_kpi.py`, 손으로 계산한 작은 예).

**임시 정답 30장 + 오늘 로그로 실행한 결과 (최종 KPI 아님):**

| 지표 | 기준 | 시차 보정 끔 | 시차 보정 켬 |
|---|---|---|---|
| 위치 MAE | ≤ 2 mm | 1.86 mm | 2.43 mm |
| 각도 MAE | ≤ 3° | 6.34° | 6.34° |
| 분류 정확도 | ≥ 95 % | 100 % | 100 % |
| 송신 성공률 (가상 카메라 입력 100회) | 100 % | 100 % | |
| 검출→송신 P95 (가상 카메라 입력) | ≤ 300 ms | 28.6 ms | |
| Pick 성공률 (11-20 개발 중 로그 전체 38개) | ≥ 90 % | 89.5 % (34/38) | |

- 위치·각도는 가영 님 평가를 재현했다. 시차 보정 해석은 WEEK3 동경 11-20 "실물 정답 30장" 참고 (정답에도 시차가 들어 있어 이 데이터로는 판단 불가).
- Pick 은 오늘 고치기 전 실패까지 다 섞인 값이다. 최종은 수정 후 반복 시험만 따로 모아 잰다.
- 송신은 가상 카메라 입력이다. 공식 값은 실물 폰 입력으로 다시 잰다.


### 2026-11-20 초록 자세 시트 (그림자 대책, 다음 주 재촬영용)

흰 시트에서는 은색 나사 옆 그림자를 같이 검출해 위치가 한쪽으로 치우쳤다 (WEEK3 동경 11-20, x −1.26 mm). 가상 카메라 바닥색 비교처럼
초록 바탕 + `metal_hsv` 면 그림자가 빠진다. 가영 님 자세 시트에 `--green` 옵션을 추가했다 (흰 시트는 그대로).

```bash
PYTHONPATH=src/vision_pick_place python3 scripts/collect_ground_truth.py --print-sheet data/ground_truth/pose_sheet_green.pdf --green
# 검출은 params.yaml 위에 params_green.yaml 을 겹쳐 쓴다 (metal_hsv, 마커 여백 9 mm, 가장자리 여백 8 mm)
ros2 run vision_pick_place vision_node --ros-args --params-file src/vision_pick_place/config/params.yaml \
    --params-file src/vision_pick_place/config/params_green.yaml
# 수집·평가 스크립트도 --params 에 두 파일을 순서대로 (뒤 파일이 덮음)
G="src/vision_pick_place/config/params.yaml src/vision_pick_place/config/params_green.yaml"
python3 scripts/collect_ground_truth.py --rotate 270 --method print --params $G
PYTHONPATH=src/vision_pick_place python scripts/evaluate_kpi.py --truth data/ground_truth/<폴더>/poses.csv --params $G
```
초록 설정은 초록 시트 전용이다. 흰 시트 사진에 쓰면 흰 바탕 전체가 '채도 낮음' 으로 한 덩어리가 되어 검출이 안 된다.

| 흰 시트와 다른 점 | 이유 |
|---|---|
| 마커 둘레 흰 여백 4 → 8 mm | 초록 바탕에서는 흰 여백 테두리도 사각형으로 보여, 4 mm 면 ArUco 가 마커를 버린다 (Gazebo 에서 0개 검출 확인) |
| 자세 선·번호·100 mm 선을 진한 초록 | 회색 선은 채도가 낮아 metal_hsv 에서 부품으로 잡힌다 |
| 초록은 가장자리 6 mm 안쪽부터, 10 mm 격자 없음 | 프린터가 가장자리를 못 칠해 흰 띠가 남는다 → 검출 영역에서 8 mm 를 뺀다 |

확인 (PNG 를 카메라 배율 2.6·3.0 px/mm 로 줄여서): PDF 297.0 × 210.0 mm, 마커 4/4 (보정 RMS 0.05~0.07 mm), 초록 설정 오검출 0개 (흰 시트 설정이면 1개).
남은 확인: 실제 인쇄본의 초록 색·조명에서 `hsv_max_saturation` (70) 이 맞는지 폰으로 한 장 찍어 본다. 인쇄 초록이 연하면 채도가 낮아질 수 있다.

### 2026-11-20 GUI 에 Gazebo 전경, Gazebo 창 없이 시연 (FPS)

- `workcell.sdf` 에 시연용 **전경 카메라** (`overview_camera`, 960×540, 5 Hz): 로봇 앞 오른쪽 위 3/4 시점에서 로봇·작업 매트·상자 두 개를 한 화면에.
  후보 6곳을 Gazebo 에서 실시간으로 옮겨 보고 골랐다. GUI ① 에 [검출 카메라] / [Gazebo 전경] 탭 (11-10 목업의 Gazebo 판을 GUI 안으로).
- 이제 Gazebo 창·RViz 를 띄우지 않아도 집기 과정을 볼 수 있다. WSL2 에서 Gazebo 창 렌더링이 가장 무거워서 끄면 빨라진다:

| | Gazebo 창 + RViz | 둘 다 끔 |
|---|---|---|
| 검출 결과 주기 (실제 처리) | 3.0 Hz | **4.1 Hz** (상한 5 Hz) |
| CPU 유휴 | 26 % | **43 %** |

```bash
SIM_ARGS="gazebo_gui:=false launch_rviz:=false" logs/start_demo.sh   # 권장
```
- GUI ③ 콘솔의 ACK 는 1초마다 한 줄로 묶는다 (`ACK  seq 120~124  5건, 평균 14 ms`). 물체가 없어도 계속 송신·ACK 되므로 한 줄씩 쓰면 집기 결과가 밀려 올라갔다. ERR·SIM 이 오면 모아 둔 ACK 를 먼저 써서 순서를 지킨다.
- 안 보이는 탭의 영상은 변환하지 않는다 (CPU 절약).

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

> 11-23 변경: 흰 시트 대신 **초록 시트** 로 한다 (아래 11-23 기록). `pose_sheet.pdf`(흰 시트)는 쓰지 않는다.

1. `pose_sheet_green.pdf` 를 100% 배율로 인쇄, 100 mm 선 확인 → 지금 시트 자리에 테이프로 고정.
2. 폰으로 한 장 찍어 초록 바탕이 `params_green.yaml` 의 `hsv_max_saturation`(70) 로 배경 처리되는지 확인 (인쇄 초록이 연하면 조정 필요, 동경 님과 공유).
3. 시트 사진 → `compute_homography.py --aruco` → `homography.npz` 갱신.
4. `collect_ground_truth.py --rotate 270 --method print --params $G` 로 30장 (`G="src/vision_pick_place/config/params.yaml src/vision_pick_place/config/params_green.yaml"`). 부품을 인쇄된 선·십자에 맞춤.
5. `evaluate_kpi.py`(팀원 1) 에 `poses.csv` 와 같은 `--params $G` 를 넘김.

### 2026-11-23 초록 시트로 결정, `pose_sheet_green.pdf` 생성

흰 시트에서 은색 나사 그림자가 같이 검출되어 위치가 치우치는 문제(WEEK3 동경 11-20, x −1.26 mm) 때문에 최종 정답은 초록 시트로 수집하기로 했다.

```bash
PYTHONPATH=src/vision_pick_place python3 scripts/collect_ground_truth.py --print-sheet data/ground_truth/pose_sheet_green.pdf --green
```

| 항목 | 결과 |
|---|---|
| 출력 | `data/ground_truth/pose_sheet_green.pdf` (+ `.png`, 12 px/mm) |
| 크기 | PDF 297.0 × 210.0 mm (A4 가로) |
| 카메라 배율 확인 (PNG 를 2.6·3.0 px/mm 로 줄임) | 마커 4/4 검출, 시트 호모그래피 RMS 0.043 mm (2.6) / 0.027 mm (3.0) |
| 오검출 (자세 선·번호가 부품으로 잡히는지) | 초록 설정(`params.yaml` + `params_green.yaml`) **0개**, 흰 시트 설정만 쓰면 1개 → 초록 시트에는 반드시 초록 설정을 겹쳐 쓴다 |
| 남은 확인 | 실제 인쇄본 색·조명에서 채도 (위 다음 주 2번) |
| 빈 초록 시트 | 자세 선(나사·너트·번호) 없이 마커·100 mm 선만 있는 판 (`render_pose_sheet(..., plan=[], green=True)`, `green_sheet_blank.pdf`). 시연·자유 배치용. 같은 확인: 297.0 × 210.0 mm, 마커 4/4, 초록 설정 오검출 0개 |
