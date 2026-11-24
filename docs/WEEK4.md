# 4주차 — KPI · Pick & Place 통합 · 실측 정답·시연

문서 구성: **1. 현황 요약 → 2. 실행 방법 → 3~5. 팀원별 기록** (WEEK3.md 와 같은 틀).
팀원 1·2 절은 각자 채운다. 이 파일은 2026-11-20 팀원 3 이 처음 만들었다.

## 1. 현황 요약 (2026-11-20)

| 팀원 | 4주차 담당 (PROJECT.md 2.3) | 상태 | 남은 일 |
|---|---|---|---|
| 팀원 1 (동경) | KPI: `evaluate_kpi.py`, 100회 송신 테스트 + 모델 비교 | 🟡 `evaluate_kpi.py` 완성 (11-20). 임시 정답 30장으로 가영 님 결과 재현 (위치 1.86 mm, 각도 6.34°) | 인쇄 시트 정답으로 최종 KPI, 실물 폰 입력 100회 송신·P95, 초록 시트 PDF, 자체 사진 모델 재평가 |
| 팀원 2 (성민) | Pick & Place 한 사이클 통합 (`pick_place_node`) | 11/23: 10개 확장, 기존 물체 20/20, 외형 개선 9/9·123.17초 | 종료 native 오류 등 제한은 TEAM2_PROGRESS 참고 |
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

### 2026-11-20 시연 구성: 왼쪽 실물 카메라 · 오른쪽 Gazebo, 실물 배치 자동 반영

11-10 목업의 두 판(GUI 대시보드 + Gazebo 창)을 한 창에 넣었다. 위 두 칸이 크게, 아래 ② 좌표 ③ 송수신 ④ 캘리브.

| 위치 | 내용 |
|---|---|
| 왼쪽 ① | **실물 휴대폰 카메라** + 검출 박스 (`/vision/debug_image`) |
| 오른쪽 ⑤ | **Gazebo 시연** 전경 카메라 (`/sim/overview/image`), 반영 상태 한 줄 |

**실물 → Gazebo 자동 반영** (헤더 체크박스, 실물 모드 기본 켬): 작업 시트 위 배치가 4패킷(약 1초) 동안 3 mm 안에서 그대로면
`/sim/spawn_objects` 로 같은 위치에 가상 부품을 만든다. 손이 들어와 흔들리는 동안은 반영하지 않고, 이미 반영한 배치에서 5 mm 넘게 바뀌어야 다시 반영한다.
다 치우면 `/sim/clear_objects`. 사이클 중에는 멈추고, 사이클 뒤 실물이 그대로면 다시 만들지 않는다. 그다음 [사이클 실행] 으로 집는다.

```bash
logs/start_demo.sh                                   # 실물: 휴대폰 (params.yaml 의 camera_url)
VISION_ARGS="-p camera_rotate_deg:=270" logs/start_demo.sh   # 폰이 세로로 보낼 때
MODE=sim logs/start_demo.sh                          # 가상 카메라 입력 ([부품 놓기 (무작위)])
IMAGE_DIR=<사진 폴더> CALIB_DIR=<보정 폴더> logs/start_demo.sh   # 폰 없이 사진으로 시험
```

확인 (폰 대신 가영 님 실물 사진, 그 사진으로 만든 보정 `data/calib/test_real`):
- 너트 사진 → Gazebo 에 너트 (0.468, −0.027) m = 시트 (168.9, 123.6) mm 와 일치.
- 사진을 나사 사진으로 바꿈 → 이전 부품을 지우고 나사 (0.528, 0.011) m, 19.4° 로 다시 스폰 (검출 (227.8, 161.0) mm).
- [사이클 실행] → 나사 1/1 상자 투입.
- 남은 것: 실제 폰 영상으로 같은 흐름 (손으로 놓는 동안 반영 안 되는지), 전경 화면에서 부품이 작게 보임 (40 mm 나사).

### 2026-11-20 GUI 위 칸 3개, 부품 모양, 부품 슬롯 길들이기

- **GUI (실물 모드)**: 위 칸을 ① 실물 카메라 · ⑤ Gazebo 전경 · ⑥ **Gazebo 위 카메라** (`/sim/camera/image` 원본) 3개로. ① 과 ⑥ 을 나란히 보면 실물 배치가 같은 자리로 반영됐는지 바로 보인다. 가상 모드에서는 ① 이 이미 Gazebo 위 카메라라 ⑥ 은 숨긴다.
- **부품 모양**: 나사는 원기둥만 있어 분필처럼 보였다. 11-20 에 머리·육각 메시를 만들었으나, 성민 님 11-23 프로필(나사산·육각 머리·구멍 너트, `team2_demo_profile.py`)과 겹쳐 11-24 에 원본 모델로 되돌리고 성민 님 모델을 쓴다.
  가상 카메라 검출: 6개 모두 종류 맞음, 너트 중심 0.5 mm 안, 나사 중심은 머리 쪽으로 약 2 mm·각도 1~2° (머리 때문에 윤곽이 한쪽으로 치우침, 실물 나사와 같은 현상).
- **부품이 수 m 날아감**: 슬롯(obj_0~2)을 처음 쓸 때 DetachableJoint 가 부품을 패드에 자동으로 붙였다 떼는데, 로봇이 대기 자세(매트 옆)에 있을 때 그러면 나사 2개가 (3.9, 1.3), (−0.8, 6.6) m 로 날아갔다. 두 번째부터는 제자리.
  → `start_demo.sh` 가 띄운 직후(로봇 시작 자세) 모든 슬롯을 한 번 놓고 지운다 (`/sim/warmup_slots`, `max_objects` 개).

### 2026-11-24 성민 님 10슬롯과 합치기

- 동경이 따로 늘린 슬롯 10개(`ur5e_vacuum.urdf.xacro`, `sim.yaml`)는 `team2_ten_slots.urdf.xacro` 와 겹쳐 `obj_3`~`obj_9` 플러그인이 2개씩 생겼다 → 되돌리고 성민 님 10슬롯·22 cm 상자를 쓴다.
- `start_demo.sh` (로컬)가 `make_team2_demo_profile.py --realistic` 으로 프로필을 만들어 넘긴다. 집기 방식은 가상 모드 `existing`, 실물 모드 `respawn` (`SCENE=` 로 바꿈).

| 확인 (headless, GUI offscreen) | 결과 |
|---|---|
| 전체 테스트 | 222개 통과 (성민 님 `0d8c130` 까지 받은 뒤), 슬롯 플러그인 `obj_0`~`obj_9` 각 1개 |
| 가상 모드 `existing`, 무작위 3개 | 3/3 투입 (`f9daf64`, `0d8c130` 에서 각 1회) |
| 실물 모드 `respawn`, 폰 대신 사진 gt015 (나사), 사진에 맞는 보정 | 검출 (227.9, 161.0) mm 19.5° / 임시 정답 (229.6, 162.1) mm 28.6°, 1/1 투입 (`f9daf64`, `0d8c130` 에서 각 1회) |
| 같은 사진, 다른 날의 보정 (시험 실수) | **마커를 나사로 검출했는데도 1/1 투입.** `respawn` 은 검출 위치에 부품을 새로 만들어 집으므로 검출이 틀려도 성공으로 나온다 → Pick 성공률 근거로 쓰지 않는다 |

남은 확인: 폰을 연결한 실물 모드, 실물 모드 `existing`, 부품 10개 사이클.

### 2026-11-24 재학습용 라벨 도구 `gui/label_obb.py` (계산은 `obb_labels.py`)

규칙 검출로 초안 회전 상자를 만들고 사람이 고친 뒤 YOLO-OBB 학습 데이터로 내보낸다. 클래스는 나사·너트 2종.

```bash
source env.sh
python3 scripts/live_view.py rtsp://172.20.10.1:8554/live data/captures/green/train   # 촬영 (val, test 도 폴더만 바꿔서)
python3 -m vision_pick_place.gui.label_obb data/captures/green                         # 라벨
python3 -m vision_pick_place.gui.label_obb data/captures/green --status                # 진행 상황
python3 -m vision_pick_place.gui.label_obb data/captures/green --export data/models/green_yolo_obb
```

- **확인 완료([Enter])가 아닌 사진이 하나라도 있으면 내보내지 않는다.** 초안이 정답으로 섞이거나 빠뜨린 부품이 배경으로 학습되는 것을 막는다. 고치면 다시 확인 전이 된다. 같은 파일이 두 폴더에 있어도 막는다.
- 나사·너트가 아닌 물체는 라벨하지 않는다. 상자 0개로 확인한 사진은 배경 사진으로 내보낸다.
- 초안 영역은 사진 속 마커로 구하므로 보정 파일이 필요 없다. 초안 기본 설정은 초록 시트용이다 (흰 시트는 `--params src/vision_pick_place/config/params.yaml`).
- 확인: 단위·창 테스트 13개, 흰 시트 사진 8장 사본에서 초안 8/8개(사진당 나사 1개), 내보낸 데이터로 YOLO 1 epoch 실행(형식 확인용, 성능 아님). 초록 실물 사진과 사람이 직접 쓰는 확인은 아직이다.
- 초안 상자는 그림자까지 포함해 헐거울 수 있다. 붙은 부품은 한 상자로 나오므로 지우고 새로 그린다.

### 2026-11-24 재학습·비교 스크립트 `scripts/finetune_obb.py` (채점은 `obb_eval.py`)

```bash
P=.venv-ml/bin/python; D=data/models/green_yolo_obb        # label_obb --export 한 폴더
$P scripts/finetune_obb.py train    --data $D --dry-run    # 계획·예상 시간
$P scripts/finetune_obb.py train    --data $D              # MVTec 시작·DOTA 시작 × 사진 10·20·40장
$P scripts/finetune_obb.py evaluate --data $D --draw       # 검증 사진 비교 (설정을 고를 때)
$P scripts/finetune_obb.py evaluate --data $D --split test --draw       # 마지막에 한 번
$P scripts/finetune_obb.py evaluate --data $D --split test_new --draw   # 학습에 없던 부품만 놓은 사진 (있을 때)
```

- 사진 수가 달라도 **가중치 갱신 횟수(기본 400회)를 같게** 한다. epoch 를 같게 하면 사진이 적을수록 덜 배워 "사진 수 효과" 와 섞인다. Ultralytics 기본 `nbs=64` 는 배치 8번에 한 번만 갱신해 10장·40 epoch 이면 갱신이 10번 남짓이라 `nbs=8` 로 둔다.
- 적은 사진은 많은 사진의 부분집합이다. 끝난 학습은 건너뛰고, 라벨·사진·설정이 바뀌면 다시 학습한다 (바뀐 뒤의 모델은 비교에서도 빠진다).
- 비교: 규칙 / 재학습 전 MVTec / 학습한 모델. 모두 같은 시트 ROI, 위치(회전 IoU 0.5)와 종류가 다 맞아야 맞힘. 신뢰도 기준은 항상 val 에서 고른다. 규칙+SVM 은 아직 넣지 않았다.
- 시간(i5-12400): 1024 입력에서 학습 12장+검증 4장이 epoch 당 약 11초 (Gazebo 시연이 같이 도는 상태). 사진 10·20·36장 × 시작 2종이면 약 3~4시간.
- 확인: 단위 테스트 8개, 합성 초록 시트 그림 20장으로 학습 4개 → val·test 비교까지 실행. **흐름 확인이며 그 점수는 실물 성능이 아니다.**
- 주의: 라벨 초안이 규칙에서 나와 규칙에 약간 유리할 수 있다. 시험 부품이 100개보다 적으면 한두 개 차이는 우열로 보지 않는다.

## 4. 팀원 2 (성민) 기록

2026-11-23: 10개 슬롯·22 cm 상자 프로필, 기존 물체 20/20 투입 및 모델 ID 유지, 외형 개선·환경 무작위화 장면 9/9·123.17초 통과. 실물 시험·전체 환경 성공률 검증은 아니다. 구현 파일·실행 명령·오류 및 남은 제한은 [TEAM2_PROGRESS.md](TEAM2_PROGRESS.md#team2-20261123-demo)에 기록했다. 다른 담당 원본 대신 팀원 2 어댑터로 연결했다.

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

### 2026-11-23 스폰 시 "없는 모델 삭제" 오류 메시지 제거 (`object_spawner.py`)

성민 님 11-23 기록(TEAM2_PROGRESS "없는 모델 초기 삭제 메시지")의 원인. 부품을 놓을 때 `spawn_all()` → `clear()` 가 `obj_0 ~ obj_{n-1}` 을 있든 없든 전부 지웠다.
Gazebo 는 없는 이름에도 `true` 로 응답해 동작은 정상이지만, 서버 로그에 슬롯마다
`[Err] [UserCommands.cc:1119] Entity named [obj_k] of type [2] not found, so not removed.` 가 찍혔다 (10개 설정이면 시작할 때 10줄).

| 항목 | 내용 |
|---|---|
| 수정 | `clear()` 가 이미 구독 중인 `pose/info` 로 **지금 있는 슬롯만** 지운다 (`existing_slots`, `GzSpawner.present_slots`). `pose/info` 를 0.5 초 안에 못 받았으면(노드가 막 떴거나 시뮬레이션 정지) 있는지 모르므로 예전처럼 전부 지운다 — 이전 실행에서 남은 모델을 못 지우면 같은 이름 스폰이 실패하기 때문 |
| 확인 (headless Gazebo, 10슬롯) | 빈 장면 clear: `[Err]` **0줄** (예전 방식 같은 조건 9줄). 2개 스폰 후 1개로 다시 놓기: 0줄, 남은 슬롯 `obj_0` 만. 단위 테스트 1개 추가, `ur5e_pick_place` 테스트 59개 통과 |

### 2026-11-23 가영 담당 전체 점검 (다른 담당 코드 수정 없음)

HEAD `0b48711` + 위 스폰 수정. `colcon build` 후 headless 시뮬레이션(`sim.launch.py gazebo_gui:=false launch_rviz:=false`).

| 항목 | 결과 |
|---|---|
| 단위 테스트 전체 (`pytest src`) | **204개 통과** (성민 님 203개 + 스폰 1개). 경고 3개는 protobuf·datetime 사용 중단 예고 |
| 3주차 스폰·흡착·TF (`check_spawn_week3.py`) | **통과**. TF = 패킷 위치·각도, 미보정 패킷 거부, 팔이 지나가도 부품 안 따라옴, 150 mm 거리 진공 거부, 너트 큰 컵 거부, 링 컵 너트·5 mm 컵 나사 2개 옮김 (놓인 위치 오차 0.5~0.7 mm), 같은 이름 재스폰, 삭제 시 마커 제거. Gazebo 로그 "not found, so not removed" **0줄** |
| 2주차 `vision_node` (Gazebo 가상 카메라, `params_sim.yaml`) | 스폰한 3개 모두 검출·분류: 나사 (100.0, 104.9) 30.0°, 너트 (200.0, 149.9), 나사 (229.3, 50.8) 135.0° — 스폰 위치와 0.7 mm 안, 각도 일치. 시작할 때 첫 프레임 전 "프레임을 읽지 못했습니다" 1줄(정상) |
| 주의 | `check_spawn_week3.py` 가 끝나면 팔이 매트 위(패드 끝 100 mm)에 남아 카메라를 가린다. 그 상태로 `vision_node` 를 켜면 회색 팔을 너트 1개로 잘못 검출했다. 팔을 대기 위치(148.5, −80 mm)로 옮긴 뒤 정상 |
| `collect_ground_truth.py --print-plan` | 30개 자세 정상 출력 |
| 폰 카메라 | 미연결이라 실물 입력은 확인 못 함 (내일 인쇄 시트 수집 때 확인) |
| 종료 (Ctrl+C) | 성민 님 기록과 같은 현상 재현: `move_group` 종료 코드 −11 (`rclcpp::Executor` 소멸 중 segfault), Gazebo 5초 안에 안 꺼져 SIGTERM, `team2_pick_place_node` "Destroyable" 예외 메시지. 모두 종료 단계이고 가영 담당 노드(스폰·vision_node)에서는 오류 없음 |

### 2026-11-24 성민 님 커밋(`0d8c130`) 받은 뒤 가영 담당 재점검 (코드 수정 없음)

성민 님이 `team2_spawner_adapter.py` 로 가영 담당 `object_spawner.py` 를 상속해 쓰기 시작해서, 원본 스폰 기능이 그대로인지 다시 확인했다. `colcon build --symlink-install` (세 패키지, 새 `team2_move_group` 포함) 후 headless 시뮬레이션(`sim.launch.py gazebo_gui:=false launch_rviz:=false`, 기본 upstream 실행).

| 항목 | 결과 |
|---|---|
| 단위 테스트 (venv `pytest`) | `ur5e_pick_place` **74개**, `vision_pick_place` **146개** 통과 = **220개** (성민 님 기록과 같음) |
| `colcon test` 주의 | `vision_pick_place` 가 0개 수집·1 skip 으로 끝남. 성민 님 새 `test_team2_gui_profile.py` 가 시스템 Python 에 없는 `PySide6` 를 모듈 단위로 건너뛰면서 패키지 전체 수집이 멈춘다. venv 의 `python -m pytest` 로는 정상. 성민 님 파일이라 수정하지 않음 |
| 3주차 스폰·흡착·TF (`check_spawn_week3.py`) | **통과**. TF = 패킷 위치·각도, 미보정 패킷 거부, 팔 통과 시 부품 이동 0.0 mm, 141.7 mm 거리 진공 거부, 너트 큰 컵 거부, 링 컵 너트(오차 0.8 mm)·5 mm 컵 나사 30°(0.9 mm)·135°(0.7 mm) 옮김, 같은 이름 재스폰, 삭제 시 마커 제거. Gazebo 로그 "not found, so not removed" **0줄** |

### 2026-11-24 시연 리허설 (성민 님 `run_team2_demo.py`, 자동 2사이클)

코드 기준 `dafc853` (성민 님 `0d8c130` 포함). 리허설 도중 받은 동경 님 `526560f`(GUI 실물 모드·슬롯 길들이기)는 이미 실행 중인 프로세스에 반영되지 않았으므로 이번 결과에 포함되지 않는다. Gazebo 가상 카메라 시연이라 초록 시트는 쓰지 않았다.

```bash
python scripts/run_team2_demo.py --gui --realistic --randomize --seed 5 --objects 9 --speed .6 --acceleration .7 \
  --position-jitter-mm 4 --compact-return --cycles 2 --transport-trials 1 --out logs/rehearsal_20261124
```

| 항목 | 결과 |
|---|---|
| 집기 | 사이클 1 **9/9** (147.99초), 사이클 2 **9/9** (135.31초) = **18/18** 상자 투입. 모델 ID 유지, 매 사이클 대기 자세 복귀. 실제 카메라 검출 좌표 사용 (합성 패킷 아님) |
| 150초 목표 | 두 사이클 모두 이내 (여유: 1회차 2초, 2회차 15초) |
| 통신 | 100/100 ACK·목표 일치, P95 **22.29 ms** (300 ms 목표 통과), TCP 왕복 P95 0.96 ms |
| 가상 카메라 보정 | 마커 4/4, 재투영 RMS 0.15 mm |
| 종료 (`shutdown_ok`) | **true**. GUI → 비전 → 이동 노드 → MoveIt → Gazebo 순서로 모두 정상 종료, 강제 종료 0, 남은 프로세스 0. native 종료 오류(-11)·Destroyable 메시지 **0건** — 11-23 가영 점검 때 보였던 종료 오류가 성민 님 새 종료 관리자로 이 실행에서는 재현되지 않음 |
| 로그 검사 (`runtime_log_review`) | 자기 충돌 접촉 3건, 거부된 계획 2건 기록 (모두 회복, 집기 실패 0). Octomap 미설정 1건(알려진 메시지). Python 예외 0 |
| 범위 | 자동 실행기의 종료이며 사람이 직접 누른 Ctrl+C·GUI 버튼 시험은 아니다. seed 5 한 장면. 실물 로봇 아님 |

결과 파일: `logs/rehearsal_20261124/summary.json` (logs 는 Git 제외).
