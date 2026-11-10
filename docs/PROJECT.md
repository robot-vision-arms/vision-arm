# PROJECT 09. 로봇 픽앤플레이스용 좌표·방향 산출

> 평면 물체의 X·Y·회전각을 비전으로 추정해 가상 로봇에 송신하고, Gazebo 속 UR5e 가 그 결과로 물체를 집어 분류한다.

## 1. 개요

| 항목 | 내용 |
|---|---|
| 취지 | 카메라 좌표계(px)를 로봇 작업 좌표계(mm)로 바꾸는 로봇 비전의 핵심 과정을 직접 구현한다. 영상 처리 결과를 실제 설비가 쓸 수 있는 숫자(mm·도)로 만들어 보내고, 그 숫자로 시뮬레이터 속 로봇이 실제로 집게 하는 것이 목표 |
| 팀 | 3인. 주차마다 작업 묶음의 **주 담당을 돌아가며** 맡고, PR 은 다른 사람이 리뷰한다 (2.3). 셋 다 비전·ROS2·시뮬레이터를 한 번씩 맡는다 |
| 기간 | 4주 (과제 범위) + 이후 최종 목표까지 확장 |
| 환경 | WSL2 Ubuntu 24.04 + ROS2 Jazzy + Python 3.12 (OpenCV, PySide6) + Gazebo Harmonic + MoveIt2. 설치는 [SETUP.md](SETUP.md) |
| 저장소 | https://github.com/robot-vision-arms/vision-arm |
| 설계 결정 기록 | 받은 3단계 역할 분담안을 어떻게 반영했는지, 왜 바꿨는지: [DECISIONS.md](DECISIONS.md) |

### 목표 단계

| 단계 | 내용 | 범위 |
|---|---|---|
| **과제 필수** | ① 부품 **중심**과 **장축 방향** 추정 ② 평면 캘리브레이션으로 **px → mm** ③ 좌표 패킷을 **가상 로봇 TCP 서버**로 송신·ACK 확인 ④ GUI 4화면 ⑤ KPI 4종 측정 | 4주 안에 반드시 |
| **확장 (4주 목표)** | Gazebo 속 **UR5e** 가 MoveIt2 로 **나사·너트**를 집어 종류별 상자에 분류 | 4주 안에 시도. 1·2주차 확인(8장)에서 막히면 범위를 줄인다 |
| **최종 목표** | Gazebo 속 6축 로봇팔이 나사·너트뿐 아니라 **여러 종류의 공구**(예: 드라이버, 스패너, 육각렌치, 펜치)를 인식해 종류별로 분류 | 4주 이후 이어서. 9장 |

확장·최종 목표는 과제 필수 범위를 **대체하지 않고 그 위에 얹는다**. TCP 송신과 KPI 는 끝까지 유지한다.

## 2. 요구사항과 담당 파일

### 2.1 기능

| 기능 | 핵심 기술 | 파일 | 범위 |
|---|---|---|---|
| 전처리 | 그레이스케일, Gaussian Blur, Threshold(Otsu) | `vision_pick_place/camera_stream.py` | 필수 |
| 검출 | findContours, minAreaRect, 모멘트 | `vision_pick_place/vision_detector.py` | 필수 |
| 부품 분류 | 장축/단축 비 (나사·너트) → 형상 특징 분류기 (공구) | `vision_detector.classify_part` | 확장 |
| 왜곡 보정 | Camera Calibration (체커보드) | `scripts/calibrate_camera.py` → `calibrator.py` | 필수 |
| px → mm | Homography (기준점 4개 이상, 8개 이상 권장) | `scripts/compute_homography.py` → `calibrator.py` | 필수 |
| ROS2 발행 | rclpy 퍼블리셔 | `vision_pick_place/vision_node.py` | 필수 |
| 로봇 송신 | TCP Socket, JSON 패킷, 통신 로그 | `robot_bridge.py`, `packet.py` | 필수 |
| 가상 로봇 | TCP 서버, 수신 로그, (확장) ROS 재발행 | `robot_server.py` | 필수 |
| KPI 측정 | MAE, 성공률, P95 | `scripts/evaluate_kpi.py` | 필수 |
| 시뮬레이터 | Gazebo Harmonic + `ur_simulation_gz` (UR5e), 작업 셀 world | `ur5e_pick_place/launch/sim.launch.py`, `worlds/workcell.sdf` | 확장 |
| 좌표 변환 | 작업 평면 → `base_link`, 툴 자세 | `ur5e_pick_place/grasp.py` | 확장 |
| 물체 스폰 | `ros_gz_sim create`, 부품 SDF 모델 | `object_spawner.py`, `models/<종류>/` | 확장 |
| 모션 플래닝·집기 | MoveIt2 (`move_group` + pymoveit2), DetachableJoint 흡착 | `pick_place_node.py`, `urdf/ur5e_vacuum.urdf.xacro` | 확장 |

### 2.2 GUI 필수 화면 (PySide6, `gui/main_window.py`)

- [ ] ① 영상 + 검출 ROI + 장축 방향 화살표
- [ ] ② px·mm 좌표, 회전각, 부품 종류
- [ ] ③ 가상 로봇 송수신 기록 (+ Gazebo 를 켰을 때 `/sim/pick_log` 집기 결과)
- [ ] ④ 캘리브레이션 상태 (내부 파라미터·호모그래피 로드 여부, 재투영 오차)

### 2.3 역할 분담 (주차별 순환)

받은 역할 분담안(3단계 × 대원 A/B/C)의 작업 묶음을 그대로 쓰되, 사람을 고정하지 않고 **주차마다 돌린다**.
고정하면 대원 A 는 MoveIt 을, 대원 C 는 비전을 한 번도 만지지 않게 되기 때문이다 (자세한 비교는 [DECISIONS.md](DECISIONS.md) 2장).
팀원 1·2·3 이 누구인지는 1주차 첫 회의에서 정해 아래에 이름을 적는다.

| 주차 | 팀원 1 (   ) | 팀원 2 (   ) | 팀원 3 (   ) |
|---|---|---|---|
| 1주차 | 캘리브레이션: `calibrate_camera.py`, `compute_homography.py`, `calibrator.py` | Gazebo 구동: UR5e + MoveIt 띄우기, `workcell.sdf`, `ur5e_vacuum.urdf.xacro` | 영상 입력: `camera_stream.py`, 웹캠(usbipd), 3명 환경 점검 |
| 2주차 | MoveIt 이동: `grasp.py`, `pick_place_node` 의 지정 좌표 이동 | 검출·분류: `vision_detector.py` (MVTec 검증) | ROS2 발행: `vision_node.py`, `packet.py` |
| 3주차 | GUI: `main_window.py`, `ros_worker.py` | TCP: `robot_bridge.py`, `robot_server.py` (+ `--publish-ros`) | 스폰·흡착: `object_spawner.py`, DetachableJoint, TF |
| 4주차 | KPI: `evaluate_kpi.py`, 100회 송신 테스트 | Pick & Place 한 사이클 통합 (`pick_place_node`) | 실측 정답 30+ 자세 수집, 시연 준비 |

- 표의 사람은 **주 담당**(일정 책임)이다. 막히면 같이 붙는다.
- PR 은 그 파일을 맡지 않은 사람이 리뷰한다. 리뷰하면서 남의 작업 묶음도 익힌다.
- 4주차 Pick & Place 통합은 가장 무거워서, 팀원 1·3 이 KPI·데이터를 끝내는 대로 같이 붙는다.

## 3. 좌표·각도·시간 정의

KPI 를 재려면 "무엇을 정답으로 볼지" 가 먼저 정해져야 합니다. 셋이 같은 정의를 쓰도록 여기서 고정합니다.

### 3.1 로봇 작업 좌표계 (`work_plane`)

- 작업 평면 = 격자(모눈) 종이 평면. 단위 mm.
- **원점과 축 방향은 격자 종이 위에 표시**해서 고정한다 (예: 왼쪽 아래 교차점이 원점, 오른쪽이 +X, 위쪽이 +Y).
- 호모그래피 기준점(`grid_points.csv`)의 `x_mm, y_mm` 이 이 좌표계로 적혀 있어야 한다.
- 카메라나 종이가 움직이면 호모그래피를 다시 계산한다.

### 3.2 회전각

- 로봇 좌표계 **+X 축에서 물체 장축까지 반시계 방향** 각도, 범위 **[0°, 180°)**.
- 장축은 앞뒤 구분이 없으므로 θ 와 θ+180° 는 같은 자세. 오차는 `min(|Δ|, 180 − |Δ|)` 로 계산 (179° vs 1° → 2°).
- **mm 좌표에서 계산**한다. 픽셀 좌표는 y 축이 아래 방향이고, 카메라가 비스듬하면 각도가 찌그러지기 때문.
- `cv2.minAreaRect` 가 주는 angle 값은 OpenCV 버전마다 범위 규칙이 달라(4.5.1 전후) 그대로 쓰지 않는다.
  `cv2.boxPoints` 꼭짓점으로 긴 변 벡터를 구하고, 양 끝점을 mm 로 변환한 뒤 `atan2` 로 계산한다.
- 정사각형·원형처럼 장축이 없는 부품(너트 등)은 패킷에서 `angle_deg: null` 로 보내고 각도 평가에서 제외한다.

### 3.3 처리 시간 (검출 → 송신)

- 시작: 프레임을 읽은 직후 `t_capture` (패킷에 포함)
- 끝: `robot_bridge` 가 가상 로봇의 ACK 를 받은 시각
- 즉 전처리·검출·변환·ROS 토픽 전달·TCP 왕복이 모두 포함된다. 카메라 노출 시간은 포함하지 않는다.
- `robot_server` 는 형식 검사 직후 ACK 를 보낸다. Gazebo 로봇 동작 시간은 **포함하지 않는다** (동작은 ACK 뒤에 따로 진행).

### 3.4 작업 평면 → 로봇 베이스 (확장)

비전이 내는 좌표는 격자 종이(`work_plane`) 기준이고, 로봇은 `base_link` 기준으로 움직인다. 둘 사이에 고정 변환 하나를 둔다.

- 실제 카메라가 본 종이를 Gazebo 안의 **가상 종이 자리**(`workcell.sdf` 의 `work_mat`)에 그대로 옮겨 놓았다고 본다.
  인식한 위치·각도에 가상 부품을 스폰하고, 로봇이 그걸 집는다 (실물 → 시뮬레이터 디지털 트윈).
- 변환값은 `ur5e_pick_place/config/sim.yaml` 의 `work_plane_x, y, z, yaw_deg` 한 곳에서 정한다.
  `pick_place_node` 가 이 값으로 직접 계산하고(`grasp.work_to_base`), 같은 값으로 static TF 도 발행해 RViz 에서 확인한다.
- 툴은 항상 아래를 본다. 흡착 패드가 길쭉하므로 패드 긴 변을 물체 장축에 맞춘다.
  θ 와 θ+180° 중 현재 손목 각도에 가까운 쪽을 골라 손목을 불필요하게 돌리지 않는다 (`grasp.nearest_equivalent_yaw_deg`).

### 3.5 부품 분류

- 패킷의 `part_type` 은 분류 이름 문자열이다. 모르는 것은 `"unknown"` 으로 보내고 로봇은 집지 않는다.
- **4주 범위 (나사·너트)**: 장축/단축 길이 비로 나눈다. 비 ≤ `nut_max_aspect`(초기값 1.5) 이면 너트, 넘으면 나사 (`classify_part`).
- **최종 목표 (여러 공구)**: 길이 비 하나로는 드라이버·육각렌치처럼 둘 다 길쭉한 공구를 못 나눈다. 9장 순서로 형상 특징 분류기로 바꾼다.

## 4. 시스템 구성

```
 카메라 / 이미지 폴더
        │
 ┌──────▼─────────────── vision_node (ROS2) ───────────────┐
 │ camera_stream → vision_detector → calibrator(px→mm, 각도) │
 └──────┬───────────────────────────────┬──────────────────┘
        │ /vision/detections (String·JSON)  │ /vision/debug_image (Image)
        ├──────────────────┐               │
 ┌──────▼──────┐    ┌───────▼───────────────▼──┐
 │ robot_bridge │    │   gui (PySide6 + QThread) │◄── /robot/log, /sim/pick_log
 └──────┬──────┘    └──────────────────────────┘
        │ TCP, NDJSON
 ┌──────▼──────┐
 │ robot_server │ ── ACK ──► robot_bridge                     ← 여기까지 과제 필수 범위
 │ (가상 로봇)  │
 └──────┬──────┘
        │ --publish-ros 일 때만: /robot/targets (String·JSON)    ← 여기부터 확장 (ur5e_pick_place)
 ┌──────▼─────────────┐   pymoveit2    ┌────────────┐   ros2_control   ┌────────────────────┐
 │  pick_place_node   │ ─────────────► │ move_group │ ───────────────► │ Gazebo Harmonic     │
 │  /sim/run_cycle    │                └────────────┘                  │  UR5e + 흡착 패드   │
 │                    │ ── ros_gz_sim create (부품 스폰) ───────────────►│  작업 셀, 분류 상자 │
 │                    │ ── /vacuum/obj_k/attach·detach (ros_gz_bridge) ►│  DetachableJoint    │
 └────────────────────┘                                                  └────────────────────┘
```

- 노드 사이는 ROS2 토픽, 로봇 컨트롤러와는 TCP. 실제 현장에서도 로봇 컨트롤러는 ROS 가 아닌 TCP/전용 프로토콜로 붙는 경우가 많아서 이렇게 나눴습니다.
- `robot_server` 가 **실제 로봇 컨트롤러 자리**입니다. 기본은 ROS 를 쓰지 않는 순수 Python 이고,
  `--publish-ros` 를 주면 받은 패킷을 Gazebo 쪽으로 넘깁니다. 그래서 시뮬레이터를 붙여도 TCP 경로·KPI 가 그대로 남습니다.
- 로봇은 패킷이 올 때마다 움직이지 않습니다. `/sim/run_cycle` 서비스를 부르면 **그 순간의 최신 패킷 한 장**으로 한 사이클(스폰 → 집기 → 분류)을 돕니다.

### 패킷 형식 (`packet.py`)

TCP 는 메시지 경계가 없어서 **JSON 1줄 + 줄바꿈(NDJSON)** 을 패킷 1개로 봅니다.

```json
{"seq": 12, "t_capture": 1762671234.123, "calib_ok": true,
 "objects": [{"id": 0, "part_type": "screw", "x_mm": 102.4, "y_mm": 55.0, "angle_deg": 37.5, "u_px": 640.2, "v_px": 360.8},
             {"id": 1, "part_type": "nut",   "x_mm": 40.1,  "y_mm": 80.3, "angle_deg": null, "u_px": 410.7, "v_px": 288.0}]}
```

응답: `{"ack": 12, "status": "ok"}`. `ack_timeout_s`(기본 1초) 안에 응답이 없으면 실패로 기록합니다.

### Pick & Place 한 사이클 (`pick_place_node`)

1. 최신 패킷 확인. `calib_ok` 가 false 면 거부. `part_types` 에 없는 종류는 빼고 `max_objects`(3)개까지.
2. 같은 위치·각도에 가상 부품 스폰(`obj_0`~`obj_2`) → 곧바로 각 슬롯 detach.
   DetachableJoint 는 대상 모델이 나타나면 바로 붙이므로 먼저 떼어 둬야 물체가 제자리에 남는다.
3. 물체마다: 접근(10 cm 위) → 하강(2 mm 위까지) → attach → 상승 → 종류별 상자 위 → detach.
4. 결과를 `/sim/pick_log` 로 발행하고 `logs/sim_pick_*.csv` 에 기록.

흡착은 물리 마찰 대신 **고정 관절을 붙였다 떼는 방식**입니다. 작은 나사를 마찰 그리퍼로 집으면 Gazebo 에서 미끄러지기 쉬워
4주 안에 안정화하기 어렵다고 봤습니다. 공구로 넓힐 때도 무게·모양에 상관없이 같은 방식이 통합니다.

## 5. 디렉토리 구조

```
robot_vision/                   ← git 저장소 = colcon 워크스페이스
├── env.sh                      매일 source: conda 해제 → ROS2 → venv → install
├── requirements.txt            venv 용 pip 패키지 (PySide6, pytest). 나머지는 apt·rosdep
├── docs/
│   ├── PROJECT.md              이 문서
│   ├── SETUP.md                환경 구축
│   └── DECISIONS.md            설계 결정 기록 (받은 역할 분담안 비교, 검증 상태)
├── scripts/                    ROS 없이 돌리는 오프라인 도구
│   ├── calibrate_camera.py     체커보드 → intrinsics.npz
│   ├── compute_homography.py   격자 기준점 → homography.npz
│   └── evaluate_kpi.py         KPI 4종 (+ 선택 2종)
├── data/                       (git 제외, 폴더만 유지)
│   ├── calib/                  체커보드·격자 사진, *.npz
│   ├── captures/               자체 촬영 이미지
│   ├── ground_truth/           실측 정답 CSV
│   └── mvtec_screws/           참고용 MVTec Screws
├── logs/                       (git 제외) 통신·수신·집기 로그 CSV
└── src/
    ├── vision_pick_place/      [과제 필수] ROS2 ament_python 패키지
    │   ├── package.xml, setup.py, setup.cfg
    │   ├── config/params.yaml  노드 파라미터
    │   ├── launch/pipeline.launch.py
    │   ├── test/               pytest
    │   └── vision_pick_place/
    │       ├── camera_stream.py    프레임 읽기, Blur·Threshold
    │       ├── vision_detector.py  외곽선, minAreaRect, 각도 정규화, 부품 분류
    │       ├── calibrator.py       왜곡 보정, px→mm, 캘리브 상태
    │       ├── packet.py           JSON 패킷 형식
    │       ├── vision_node.py      [노드] 검출 결과 발행
    │       ├── robot_bridge.py     [노드] 토픽 → TCP 송신, ACK·지연 기록
    │       ├── robot_server.py     가상 로봇 TCP 서버 (--publish-ros 로 Gazebo 연동)
    │       └── gui/
    │           ├── main_window.py  [노드] GUI 4화면
    │           └── ros_worker.py   ROS 구독용 QThread
    └── ur5e_pick_place/        [확장] Gazebo UR5e 픽앤플레이스 패키지
        ├── package.xml         ur_simulation_gz, MoveIt, pymoveit2, ros_gz 의존성 (rosdep 으로 설치)
        ├── config/sim.yaml     작업 평면 위치, 분류 종류·상자 위치, 접근 높이
        ├── launch/sim.launch.py  Gazebo + UR5e + move_group + RViz + 브리지 + 노드
        ├── urdf/ur5e_vacuum.urdf.xacro  기본 UR5e + 흡착 패드 + DetachableJoint 슬롯 3개
        ├── worlds/workcell.sdf 바닥, 가상 종이 자리, 분류 상자
        ├── models/<종류>/      부품 모델 (screw, nut → 최종 목표에서 공구 추가)
        ├── test/test_grasp.py
        └── ur5e_pick_place/
            ├── grasp.py            work_plane → base_link, 툴 자세 (순수 계산)
            ├── object_spawner.py   가상 부품 스폰·삭제
            └── pick_place_node.py  [노드] /sim/run_cycle 로 한 사이클 실행
```

**분류 종류를 하나 늘리는 법** (코드 수정 없음): `models/<종류>/model.sdf` 추가 → `sim.yaml` 의 `part_types`, `bin_x`, `bin_y` 에 한 칸씩 추가 → `workcell.sdf` 에 `<종류>_bin` 추가 → `colcon build`.

## 6. 데이터

### 6.1 자체 촬영 (메인)

나사·너트(최종 목표에서는 공구까지)를 격자 종이 위에 놓고 촬영합니다. **좌표·각도 정답은 이 데이터로만 만듭니다.**

- 격자 종이에 목표 자세(중심점 + 각도선)를 미리 그려 두고 부품을 그 위에 맞춰 놓으면 실측값 = 그린 값이 됩니다.
- 위치는 작업 영역 전체에 고르게, 각도는 0~180° 를 고르게 (예: 0, 15, 30, … 165°).
- 최소 30 자세 이상 권장 (MAE 가 몇 개 샘플에 휘둘리지 않도록).
- `data/ground_truth/poses.csv` 형식: `sample_id, image, x_mm, y_mm, angle_deg, part_type`

### 6.2 MVTec Screws (참고)

| 항목 | 내용 |
|---|---|
| 구성 | 나사·너트 13종, 384장, 회전 BBox 4,427개 (README 표기는 4,426), COCO 유사 형식 (박스당 파라미터 5개) |
| 받는 곳 | Kaggle [ipythonx/mvtec-screws](https://www.kaggle.com/datasets/ipythonx/mvtec-screws) (공식 사이트: [mvtec.com](https://www.mvtec.com/company/research/datasets/mvtec-screws)) |
| 용도 | 검출·각도 알고리즘을 **픽셀 기준**으로 먼저 검증 (촬영 전 2주차 개발용). 13종이라 다중 분류 연습(9장 1단계)에도 쓴다 |
| 한계 | 로봇 세계좌표(mm) 정답이 아님 → 위치 KPI 에는 사용 불가. 공구는 없음 |
| 라이선스 | CC BY-NC-SA 4.0 (비상업) |

라이선스상 재배포하지 않으므로 **저장소에 올리지 않고 각자 받습니다** (`.gitignore` 의 `data/**`). 둘 중 하나:

```bash
# A. Kaggle CLI (SETUP.md Step 2 의 API 토큰 필요)
kaggle datasets download -d ipythonx/mvtec-screws -p data/mvtec_screws --unzip

# B. 브라우저로 Kaggle 에서 zip 다운로드 → data/mvtec_screws/ 에 압축 해제 (zip 안에 상위 폴더 없음)
```

풀고 나면 이렇게 됩니다:

```
data/mvtec_screws/
├── README_v1.0.txt             데이터셋 설명 (형식·라이선스)
├── images/                     screws_001.png ~ screws_384.png
├── mvtec_screws.json           전체 어노테이션 (COCO 유사 형식)
├── mvtec_screws_{train,val,test}.json   논문에서 쓴 분할 (이미지 269 / 55 / 60장)
└── *.hdict                     HALCON 전용 형식 (안 씀)
```

확인: 이미지 384장, 범주 13개(`type_001`~`type_013`). 어노테이션은 README 에 4,426개로 적혀 있지만
`mvtec_screws.json` 에는 **4,427개**가 들어 있습니다 (분할 3개 합도 3,119 + 647 + 661 = 4,427). 2026-11-10 에 받은 Kaggle 판 기준.

**박스 형식이 일반 COCO 와 다릅니다.** `bbox = [row, col, width, height, phi]`

- `row, col`: 박스 **중심**의 행(세로)·열(가로) 좌표. COCO 의 `x, y`(왼쪽 위 모서리)가 아니고 순서도 반대
- `width`: 박스 방향과 **평행한** 길이 (= 장축), `height`: 수직 길이
- `phi`: 라디안, 이미지 가로축에서 **반시계(위쪽이 +)**, 범위 (−π, π]. 픽셀 v 축은 아래 방향이라 `atan2` 로 비교할 때 부호에 주의
- 우리 각도 정의(3.2)와 비교하려면 `degrees(phi) % 180` 으로 [0°, 180°) 로 접는다

## 7. 성능 평가 (KPI)

| 지표 | 측정 방법 | 기준 | 범위 | 결과 |
|---|---|---|---|---|
| 위치 오차 (MAE) | 격자 실측 대비 (x, y) 유클리드 거리 평균 | ≤ 2 mm | 필수 | 미측정 |
| 각도 오차 (MAE) | [0°, 180°) 정의, 3.2 의 오차식. 장축 없는 부품 제외 | ≤ 3° | 필수 | 미측정 |
| 송신 성공률 | 100회 연속 송신 중 ACK 수신 비율 | 100 % | 필수 | 미측정 |
| 검출→송신 시간 | 3.3 구간, 100회 이상 측정한 P95 | ≤ 300 ms | 필수 | 미측정 |
| 분류 정확도 | `part_type` 예측 = 실측 비율 | ≥ 95 % | 선택 | 미측정 |
| Pick 성공률 | Gazebo 사이클에서 물체가 맞는 상자에 들어간 비율 (20개 이상) | ≥ 90 % | 선택 | 미측정 |

기준값은 교육용 권장 목표입니다. **실측 전에는 달성으로 표기하지 않습니다.**
Pick 성공률은 시뮬레이터 안의 결과라 비전 정확도와 별개로 적습니다 (스폰 위치 = 인식 위치라서 비전 오차는 반영되지 않음).

### 측정 장비 (측정할 때 채움)

| 항목 | 값 |
|---|---|
| CPU | Intel Core i5-12400 (6코어 12스레드) ※ 팀원 1 PC, WSL2 |
| GPU | 미사용 (OpenCV CPU 처리). Gazebo 렌더링은 WSLg 기본 |
| RAM | |
| 카메라 / 해상도 / FPS | |
| 카메라 높이·작업 영역 크기 | |
| OS / ROS / OpenCV / Gazebo 버전 | Ubuntu 24.04 (WSL2) / Jazzy / / Harmonic |

## 8. 4주 로드맵

| 주차 | 할 일 | 끝났다는 기준 |
|---|---|---|
| **1주차** 환경·캘리브레이션·시뮬 구동 | 전원 SETUP.md 완료 (웹캠, Gazebo·MoveIt 포함) · 체커보드 캘리브레이션 · 카메라 고정 후 격자 호모그래피 · 로봇 좌표계(원점·축) 종이에 표시 · **UR5e 를 Gazebo 에 띄우고 MoveIt 으로 움직이기** | 3명 모두 `colcon build` 성공 · `intrinsics.npz`, `homography.npz` 생성 · 재투영 오차(mm) 기록 · **확인 ①** 통과 여부 기록 |
| **2주차** 비전·ROS2 노드·로봇 이동 | MVTec 으로 검출·각도·분류 먼저 검증 · 자체 부품 촬영 · Blur/Threshold 조정 · `vision_node` 발행 · 코드(pymoveit2)로 작업 평면 위 지정 좌표 이동 · 손으로 스폰한 부품 attach/detach 시험 | `ros2 topic echo /vision/detections` 로 mm·각도·종류 확인 · `colcon test` 통과 · **확인 ②** 통과 여부 기록 |
| **3주차** GUI·TCP·스폰 | 메인 윈도우 4화면 · QThread 로 ROS 구독 · `robot_server`, `robot_bridge` · `--publish-ros` · 인식 결과대로 Gazebo 에 부품 스폰 | GUI 실시간 표시 · TCP ACK 수신 · 카메라 앞 부품 위치대로 Gazebo 에 부품이 생김 |
| **4주차** Pick & Place·KPI·시연 | 한 사이클 완성 · 실측 정답 30+ 자세 · 100회 송신 테스트 · `evaluate_kpi.py` | KPI 표 필수 4칸 + 측정 장비 표 채움 · 시연 (카메라 → GUI → TCP → Gazebo 분류) |

### 확인 지점 (Go / No-Go)

시뮬레이터 쪽은 WSL2 에서 막히기 쉬워서, 늦게 알지 않도록 앞쪽 주차에 확인 지점을 둡니다.

| 확인 | 시점 | 통과 기준 | 실패하면 |
|---|---|---|---|
| ① 구동 | 1주차 끝 | `ros2 launch ur5e_pick_place sim.launch.py` 로 Gazebo·RViz 가 뜨고, RViz MotionPlanning 에서 Plan & Execute 하면 Gazebo 속 UR5e 가 움직인다 | SETUP.md Step 5 의 대체 방법(소프트웨어 렌더링, `gazebo_gui:=false`) → 그래도 안 되면 Gazebo 를 빼고 과제 필수 범위로 진행, MoveIt 은 RViz 에서 궤적 계획만 시연 |
| ② 흡착 | 2주차 끝 | 손으로 스폰한 부품을 attach → 팔을 움직이면 따라오고 → detach 하면 떨어진다 | 대체 플러그인(DECISIONS.md 4장)을 시험 → 안 되면 "부품 위치까지 이동 → 상자까지 이동" 궤적만 시연하고 부품은 `set_pose` 로 옮긴다 |

## 9. 최종 목표: 여러 종류의 공구 분류 (4주 이후)

4주 결과(나사·너트 분류)가 끝난 뒤 같은 구조로 넓힙니다. 파이프라인과 패킷 형식은 그대로고, 바뀌는 건 **분류기·부품 모델·상자 수**입니다.

| 순서 | 할 일 | 끝났다는 기준 |
|---|---|---|
| 1 | **형상 특징 분류기**: 윤곽선마다 면적(mm²), 장축 길이(mm), 길이 비, 볼록도(solidity), 구멍 수(윤곽선 계층), Hu 모멘트를 뽑아 규칙 → k-NN/SVM 순서로 비교. MVTec 13종으로 먼저 연습 | 분류 정확도 표 (종류별 혼동 행렬) |
| 2 | **공구 촬영 데이터**: 대상 공구 4~6종 정하기 (예: 드라이버, 스패너, 육각렌치, 펜치), 종류당 30장 이상, `part_type` 라벨 | `poses.csv` 에 공구 행 추가 |
| 3 | **집는 점**: 공구는 무게중심이 몸체 밖에 떨어질 수 있음 (펜치, 트인 스패너) → 거리 변환 최댓값(물체 안쪽에서 가장 두꺼운 점)을 집는 점으로 | 모든 공구에서 집는 점이 마스크 안 |
| 4 | **Gazebo 모델·상자**: 공구마다 `models/<종류>/model.sdf` (기본 도형 조합으로 단순화, 메시는 라이선스 확인 후), 상자를 베이스 둘레에 배치 | 5장 "종류 늘리는 법" 대로 추가해 Pick 성공률 측정 |
| 5 | (선택) 규칙·k-NN 으로 정확도가 안 나오면 학습 기반 회전 박스 검출(OBB)과 비교. 쓰게 되면 학습 데이터·추론 시간을 KPI 표에 같이 적는다 | 방식별 정확도·처리 시간 비교표 |

- 상자가 늘면 UR5e 도달 범위(약 0.85 m) 안에 다 들어가는지, 상자끼리 계획 경로가 막히지 않는지 RViz 에서 먼저 확인합니다.
- 공구 수가 늘어도 집는 방식(흡착 고정 관절)은 같습니다. 최종 목표에서 마찰 그리퍼로 바꾸려면 별도 과제로 봅니다.

## 10. 1인 축소안 (참고)

부품 1종의 XY·각도만 추정, 송신 대신 CSV 기록, GUI 는 영상+좌표만. 3인 팀이므로 풀 스펙으로 진행하되,
일정이 밀리면 줄이는 순서는 **Gazebo 집기 → Gazebo 스폰 → (필수 범위 유지)** 입니다. TCP 송신과 KPI 는 마지막까지 남깁니다.

## 11. 미리 알아 둘 위험

| 위험 | 대응 |
|---|---|
| WSL2 에서 웹캠이 안 잡힘 | usbipd-win (SETUP.md Step 7). 안 되면 Windows 에서 촬영 → `image_dir` 모드 |
| 조명·그림자로 이진화 실패 | 배경을 부품과 대비되는 단색으로, 조명 고정. Otsu 로 시작 후 고정값 비교 |
| 카메라가 비스듬해 mm 오차 증가 | 카메라를 작업면에 최대한 수직으로 고정, 호모그래피 기준점은 작업 영역 가장자리까지 넓게 |
| 카메라·종이가 움직임 | 테이프로 고정, 움직였으면 호모그래피 재계산 (GUI 캘리브 상태에 계산 시각 표시) |
| 원형·정사각형 부품 | 장축이 없어 각도가 불안정 → `angle_deg: null`, 각도 평가 대상에서 제외하고 문서에 명시 |
| WSL2 에서 Gazebo 가 느리거나 화면이 깨짐 | 확인 ① 에서 일찍 발견. 소프트웨어 렌더링, Gazebo 창 끄고 RViz 만 보기, `max_velocity_scaling` 낮추기 |
| Gazebo + MoveIt + RViz 메모리 부족 | WSL 메모리 8 GB 이상 (SETUP.md Step 5), 브라우저 등 정리 |
| 흡착(DetachableJoint)이 늦게 스폰한 모델을 못 잡음 | 확인 ② 에서 일찍 발견. 대체 플러그인 또는 `set_pose` 방식 (8장) |
| 작은 부품이 물리 엔진에서 튐 | 부품 질량·관성을 실제 비율로 넣었음. 튀면 패드를 2 mm 위에서 멈춰 접촉 없이 붙이는 현재 방식을 유지하고, 상자에는 낮은 위치에서 놓기 |
| 공구 종류가 늘면 길이 비 규칙으로 분류 불가 | 9장 1단계 형상 특징 분류기. 4주 범위는 나사·너트 2종으로 고정 |

## 12. 참고 자료

- [OpenCV Camera Calibration 튜토리얼](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- [OpenCV Contour Features (minAreaRect)](https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html)
- [ROS2 Jazzy 설치 (Ubuntu deb)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [MVTec Screws 공식 페이지](https://www.mvtec.com/company/research/datasets/mvtec-screws)
- [ur_simulation_gz 설치](https://docs.ros.org/en/rolling/p/ur_simulation_gz/doc/installation.html) · [사용법](https://docs.ros.org/en/rolling/p/ur_simulation_gz/doc/usage.html)
- [Gazebo Sim DetachableJoint (Harmonic = gz-sim 8)](https://gazebosim.org/api/sim/8/detachablejoints.html)
- [pymoveit2 (Jazzy)](https://docs.ros.org/en/jazzy/p/pymoveit2/) · [MoveIt2 바이너리 설치](https://moveit.ros.org/install-moveit2/binary)
