# 설계 결정 기록

받은 "UR5e 기반 ROS2 & Gazebo 픽앤플레이스 3인 역할 분담표"를 이 저장소에 어떻게 반영했는지, 무엇을 바꿨고 왜 바꿨는지 남깁니다.
각 결정은 **검증 상태**를 같이 적습니다. "문서 확인" 은 공식 문서를 읽고 확인한 것, "미확인" 은 아직 직접 돌려 보지 않은 것입니다.
(작성 및 환경 재점검: 2026-11-10. ROS2 Jazzy와 시뮬레이터 패키지 설치 완료. 실제 구동에서 발견한 설치 차단 항목은 아래에 기록합니다.)

### 현재 PC 실행 점검 (2026-11-10)

- Python·ROS import, pip 의존성 검사 정상. 테스트 7개 중 3개 통과, 4개 미구현으로 skipped.
- WSL 설정: `C:\Users\user1\.wslconfig`에 memory=10GB, swap=8GB 작성. 11-13 점검 때 파일이 없어져 7.6 GiB 로 돌아가 있어 다시 작성함. 원인 미확인 (PC 초기화 프로그램 가능성). 재시작 후 적용 확인 필요.
- 이전에 누락된 `ur_robot_driver`는 Jazzy 3.9.0 설치 확인 완료. sudo 인증 성공, rosdep check와 pip check 통과. 실행 의존성을 package.xml에 추가함.
- UR 기본 예제 재실행에서 MoveIt 초기화(`You can start planning now!`) 및 joint_state_broadcaster / scaled_joint_trajectory_controller 활성화 확인. URDF·SRDF 로봇 이름 불일치와 선택적 인식/Octomap 기능 오류가 남아 있으며, 이후 기본 예제 Plan·Execute 및 Gazebo 팔 이동은 사용자 확인 완료(2026-11-13, 8절). 프로젝트 작업 셀은 미검증.
- `/dev/video*` 없음. 현재 도구 환경은 Windows PowerShell 실행이 `Exec format error`로 실패하므로 usbipd 설치 폴더는 현재 존재함. 버전·서비스·USB 장치 확인은 Windows에서 실행해야 함.
- 다음 절차: `WEEK1.md`의 현재 PC 남은 작업 참고. WSL 재시작은 현재 세션을 종료하므로 사용자가 Windows에서 수행.

### 2주차 실행 점검 (2026-11-13)

- 현재 세션에서 메모리 9.7GiB / swap 8.0GiB 확인.
- `colcon build` 2개 패키지 성공, `colcon test` 35개 통과(실패·오류·skip 0개).
- 프로젝트 작업 셀을 headless로 띄워 지정 좌표 이동, 늦게 스폰한 부품의 자동 연결·분리·재연결·상승 추종·낙하를 확인했다.
- URDF/SRDF 이름과 패드 고정 장착부 충돌 설정을 보완했다. GUI·실제 폰 보정·KPI·부품 재생성 후 재흡착은 미검증.
- 나머지 한계와 실행·평가 수치는 [WEEK2.md](WEEK2.md)에 기록했다. 위 11-10 기록은 당시 상태로 남긴다.

## 1. 받은 분담안 요약

| 단계 | 목표 | 대원 A | 대원 B | 대원 C |
|---|---|---|---|---|
| 1. 비전 & GUI | 카메라로 나사/너트를 인식해 mm 좌표·각도를 화면에 표시 | `camera_stream.py` 웹캠, Blur·Threshold | `vision_detector.py` findContours, minAreaRect, 중심·장축 각도, 나사/너트 분류 | `gui_main.py` PySide6 대시보드 |
| 2. 캘리브레이션 & ROS2 & MoveIt2 | px → mm, ROS2 토픽 발행, MoveIt2 궤적 계산 | `calibrator.py` 모눈종이 실측으로 px → mm | `vision_publisher.py` X, Y, Angle, Type 토픽 발행 | `motion_planner.py` 구독 + MoveIt2 로 장애물 회피 궤적 |
| 3. Gazebo & Pick & Place | Gazebo 속 UR5e 가 나사를 집어 분류 | Gazebo 환경 (`ur_simulation_gazebo` 클론) | 비전 결과와 연동된 가상 나사/너트 모델 배치 | 그리퍼로 집어 종류별 상자에 놓는 최종 시나리오 |

## 2. 결정

| # | 받은 안 | 결정 | 이유 | 검증 상태 |
|---|---|---|---|---|
| 1 | Gazebo Pick & Place 가 프로젝트의 중심 | **과제 필수 범위(TCP 송신·ACK, GUI 4화면, KPI 4종) 위에 확장으로** 얹는다. `robot_server` 가 TCP 로 받아 ACK 한 뒤 `--publish-ros` 일 때 Gazebo 로 넘긴다 | 받은 안대로 바꾸면 과제 요구인 TCP 가상 로봇 서버와 KPI 가 빠진다. `robot_server` 를 "실제 로봇 컨트롤러 자리" 로 둔 기존 설계(PROJECT.md 4장)와도 맞는다 | 설계 결정 (검증 대상 아님) |
| 2 | `ur_simulation_gazebo` 클론 | `ros-jazzy-ur-simulation-gz` apt 설치 | `ur_simulation_gazebo` 는 Gazebo Classic(Humble) 용. Jazzy 의 짝은 Gazebo Harmonic 이고 패키지가 `ur_simulation_gz`. Gazebo Classic 은 2025년 1월 지원 종료 | apt 이름 형식·`description_file`·`world_file` 인자: 문서 확인 ([설치](https://docs.ros.org/en/rolling/p/ur_simulation_gz/doc/installation.html), [사용법](https://docs.ros.org/en/rolling/p/ur_simulation_gz/doc/usage.html), rolling 문서 기준). Classic 지원 종료: 사전 지식 기준, 오늘 따로 확인 안 함 |
| 3 | Gazebo 환경 구축이 3단계(마지막) | **1주차**에 구동 확인 ①, 2주차에 흡착 확인 ② (PROJECT.md 8장) | WSL2 에서 Gazebo 는 렌더링·메모리 문제로 막히기 쉽다. 4주차에 처음 알면 대응할 시간이 없다 | 설계 결정 |
| 4 | 그리퍼로 집기 (방식 미정) | 흡착 패드 + Gazebo `DetachableJoint` (고정 관절을 붙였다 떼기) | `ur_simulation_gz` 의 UR5e 에는 그리퍼가 없다. 마찰 그리퍼는 URDF·MoveIt 설정 추가가 크고, 작은 나사가 물리 엔진에서 미끄러지기 쉽다. 공구로 넓혀도 같은 방식이 통한다 | attach/detach 토픽 방식: 문서 확인 ([gz-sim 8 DetachableJoint](https://gazebosim.org/api/sim/8/detachablejoints.html)). 늦게 스폰한 모델에 대한 동작: **미확인** (4장) |
| 5 | MoveIt2 API (종류 미정) | `move_group`(ur_moveit_config 가 띄움) + **pymoveit2** | pymoveit2 는 move_group 의 액션·서비스를 부르는 클라이언트라 UR 기본 런치를 그대로 쓸 수 있다. 공식 `moveit_py` 는 노드 안에서 MoveIt 설정을 직접 불러와야 해서 설정이 더 많다 | pymoveit2 Jazzy 배포: 문서 확인 ([docs.ros.org jazzy](https://docs.ros.org/en/jazzy/p/pymoveit2/)). `moveit_py` 의 Jazzy apt 패키지: 확인 못 함 (conda 판만 확인) |
| 6 | 비전 좌표를 바로 로봇에 | 격자 종이(`work_plane`) → `base_link` 고정 변환을 추가 (`sim.yaml` 한 곳) | 비전 mm 는 종이 기준이라 로봇이 쓰려면 변환이 하나 더 필요하다. 받은 안에는 없었음 | 설계 결정. 수식은 `grasp.py` TODO + `test_grasp.py` (skip) |
| 7 | 각도 → 그리퍼 회전 (언급 없음) | θ 와 θ+180° 중 현재 손목 각도에 가까운 쪽 | 장축 각도가 [0°, 180°) 대칭이라, 고르지 않으면 손목을 반 바퀴 더 돌릴 수 있다 | 설계 결정 |
| 8 | 1단계에서 mm 좌표를 표시 | 캘리브레이션을 1주차로 | 받은 안은 mm 를 1단계에서 쓰는데 mm 변환은 2단계라 순서가 맞지 않았다 | 설계 결정 |
| 9 | 대원 A/B/C 에 단계마다 같은 계열 고정 | 작업 묶음은 그대로, **주차마다 사람을 돌림** + PR 교차 리뷰 (PROJECT.md 2.3) | 고정하면 A 는 MoveIt 을, C 는 비전을 한 번도 안 맡는다. "셋 다 비전·ROS2·시뮬레이터를 고루 경험" 이라는 받은 안의 목표와 팀 원칙(전원 학습)에 맞춘다 | 설계 결정 |
| 10 | `vision_publisher.py`, `motion_planner.py`, `gui_main.py` | 기존 이름 유지: `vision_node.py`, `pick_place_node.py`, `gui/main_window.py` | 같은 역할의 파일이 이미 뼈대로 있어서 이름만 바꾸면 문서·런치·entry point 를 다 고쳐야 한다 | 설계 결정 |
| 11 | 로봇이 토픽을 받는 대로 움직임 | `/sim/run_cycle` 서비스를 부르면 최신 패킷 한 장으로 한 사이클 | 비전이 10 Hz 로 발행하므로 패킷마다 움직이면 로봇이 멈추지 않는다 | 설계 결정 |
| 12 | 분류 대상 나사/너트 | 4주 범위 = 나사·너트, **최종 목표 = 여러 공구** (PROJECT.md 9장). 시뮬레이터 쪽은 종류·상자를 `sim.yaml` 목록으로 둬서 종류를 늘려도 코드는 안 바꾼다 | 팀 최종 목표가 공구 분류까지라서. 길이 비 규칙은 2종일 때만 통하므로 분류기는 나중에 교체 | 설계 결정 |
| 13 | 검출·분류 방식 (언급 없음) | 4주 기본은 OpenCV 규칙(이진화 → 윤곽선 → `minAreaRect` → 길이 비) 유지. **3·4주차에 규칙 / 형상 특징 + k-NN·SVM·RF / CNN 분류 / YOLOv8-OBB 를 같은 데이터로 비교**하고 결과로 고른다 (11절, 팀원 1 동경 담당) | 규칙은 빠르고 설명 가능하지만 MVTec 에서 접촉 물체·배경 오검출이 남는다. 딥러닝을 무조건 쓰기보다 같은 기준으로 비교한 근거로 고르는 것이 현업 방식이고 발표에도 설득력이 있다 | MVTec 비교 완료 (2026-11-19): 종합 F1 규칙 0.747, 규칙+SVM 0.804, YOLOv8n-OBB 0.989 (CPU 98 ms/장). 자체 사진 재평가·채택은 4주차 |

## 3. 받은 단계 → 주차 대응

| 받은 단계 | 들어간 주차 (PROJECT.md 8장) |
|---|---|
| 1단계 비전 (camera_stream, vision_detector) | 1주차 영상 입력, 2주차 검출·분류 |
| 1단계 GUI | 3주차 |
| 2단계 calibrator | 1주차 (순서 앞당김, 결정 8) |
| 2단계 publisher | 2주차 `vision_node` |
| 2단계 motion_planner | 2주차 지정 좌표 이동 → 4주차 사이클 |
| 3단계 Gazebo 환경 | 1주차 구동 확인 ① (앞당김, 결정 3) |
| 3단계 오브젝트 스폰 | 2주차 흡착 확인 ②, 3주차 인식 결과 스폰 |
| 3단계 Pick & Place | 4주차 |
| (받은 안에 없음) TCP·KPI | 3주차 TCP, 4주차 KPI |

## 4. 아직 확인 못 한 것 (1~2주차에 채움)

돌려 보고 결과를 이 표에 적습니다. 다르면 해당 파일을 고치고 "조치" 에 적습니다.

| 항목 | 확인 방법 | 관련 파일 | 결과 | 조치 |
|---|---|---|---|---|
| `ur_sim_control.launch.py` 가 `description_file`, `world_file`, `gazebo_gui`, `launch_rviz` 를 받는지 (Jazzy 판) | 설치된 런치 소스·실행 확인 | `launch/sim.launch.py` | 11-13 확인, 프로젝트 world·패드 포함 로봇으로 headless 구동 | XML 주석의 YAML 해석 오류 수정 |
| `ur_moveit.launch.py` 가 `ur_type`, `use_sim_time`, `launch_rviz` 를 받는지 | `--show-args` | `launch/sim.launch.py` | | |
| `ur_simulation_gz/urdf/ur_gz.urdf.xacro` 경로 | `ls` (SETUP.md 5-3) | `urdf/ur5e_vacuum.urdf.xacro` | | |
| 계획 그룹 이름 `ur_manipulator` | 설치된 SRDF·MoveIt 실행 | `pick_place_node.py` | 11-13 해당 그룹으로 실제 계획·실행 성공 | 패드를 포함한 로컬 URDF/SRDF 런치 추가 |
| 고정 관절로 붙은 `tool0` 이 SDF 변환 뒤 `wrist_3_link` 로 합쳐지는지 | `gz model -m ur -l` 로 링크 목록 | `urdf/ur5e_vacuum.urdf.xacro` | | |
| DetachableJoint 가 **나중에 스폰한** 모델을 잡는지, 스폰 즉시 붙는지 | 확인 ② (`scripts/check_sim_week2.py`) | `pick_place_node.py`, `object_spawner.py` | 11-13 통과. 자동 연결→분리→재연결→약 9.9cm 상승 추종→분리·낙하 | 슬롯별 상태 토픽과 부품 스폰 전 경고 억제 설정 추가 |
| DetachableJoint 가 지웠다 다시 만든 같은 이름 모델을 잡는지 | 사이클 두 번 연속 | `object_spawner.remove` | | |
| WSL2 에서 Gazebo + MoveIt + RViz 가 쓸 만한 속도로 도는지 | 확인 ① | `env.sh`, `sim.yaml` | | |
| rosdep 이 `ur5e_pick_place/package.xml` 의존성을 전부 찾는지 | SETUP.md Step 3 | `package.xml` | | |
| 각 팀원 PC 의 실제 설치 버전 (PySide6, Gazebo, MoveIt) | SETUP.md Step 2 확인 명령, `gz sim --version`, `apt-cache policy ros-jazzy-moveit` | PROJECT.md 7장 측정 장비 표 | | |

DetachableJoint 가 안 되면 시험해 볼 대안 (둘 다 아직 안 써 봄):
- 모델을 미리 정하지 않고 서비스로 붙였다 떼는 커뮤니티 플러그인 (예: [gz_attach_links](https://github.com/oKermorgant/gz_attach_links))
- 집은 동안 부품 위치를 툴 위치로 계속 옮기는 방식 (`/world/<world>/set_pose`)

## 5. 출처

2026-11-10 에 확인한 문서:
- [ur_simulation_gz 설치](https://docs.ros.org/en/rolling/p/ur_simulation_gz/doc/installation.html), [사용법](https://docs.ros.org/en/rolling/p/ur_simulation_gz/doc/usage.html), [마이그레이션 노트](https://docs.ros.org/en/rolling/p/ur_simulation_gz/doc/migration_notes.html)
- [Gazebo Sim 8 DetachableJoint](https://gazebosim.org/api/sim/8/detachablejoints.html)
- [pymoveit2 (Jazzy)](https://docs.ros.org/en/jazzy/p/pymoveit2/), [pymoveit2 패키지 인덱스](https://index.ros.org/p/pymoveit2/)
- [MoveIt2 바이너리 설치](https://moveit.ros.org/install-moveit2/binary)
- [ros-jazzy-moveit-py (RoboStack conda 판)](https://anaconda.org/robostack-jazzy/ros-jazzy-moveit-py)

## 6. 실시간 웹캠 연결 제외 (2026-11-10, 7절로 대체됨: 실시간 입력은 스마트폰으로)

실시간 USB 웹캠 연결·usbipd 설정만 제외합니다. 실제 모눈종이 촬영과 카메라 좌표계(px) → 로봇 작업 좌표계(mm) 변환 계획은 유지합니다.

- 보유 MVTec 사진은 물체 검출·중심점·장축 각도 계산 로직을 개발·검증하는 데이터입니다.
- 실제 카메라로 촬영한 체커보드·모눈종이·부품 사진을 파일로 가져옵니다. 격자 실측 좌표로 보정하고 같은 촬영 조건의 부품 위치·방향을 mm·도로 변환합니다.
- 보유 사진에는 mm 정답이 없으므로 임의 크기를 부여한 결과를 실제 위치 KPI로 삼지 않습니다.
- 변환 결과를 Gazebo UR5e에 전달합니다. 가상 카메라 도입은 결정하지 않았습니다.
- 이미지 입력·보정 스크립트·관련 노드는 아직 구현이 필요합니다. usbipd는 설치되어 있지만 현재 작업의 필수 항목이 아닙니다.
- 기존 역할 분담표의 웹캠 언급은 이전 계획 기록이며, 현재 범위는 이 절을 따릅니다.

## 7. 실시간 스마트폰 카메라 입력 채택 (2026-11-10, 6절 대체)

6절을 되돌려 실시간 영상을 기본 입력으로 둡니다. PC 에 웹캠이 없으므로 카메라는 스마트폰을 씁니다.

- 이유: 부품을 옮기면 GUI 좌표·각도가 바로 따라오고, `/sim/run_cycle` 을 부르면 그 순간의 위치대로 Gazebo 에 부품이 생겨 로봇이 집는 흐름을 시연에서 직접 보여 줄 수 있습니다.
- 연결: 폰 IP 카메라 앱이 Wi-Fi 로 영상 주소(HTTP/RTSP)를 열고, `cv2.VideoCapture(camera_url)` 이 그 주소를 직접 엽니다. USB 를 쓰지 않으므로 usbipd 는 필요 없습니다. 설정은 `params.yaml` 의 `camera_url` (SETUP.md Step 7).
- 웹캠 구매 대신 폰을 고른 이유: 추가 비용이 없고 팀원 모두 바로 쓸 수 있습니다. 대신 Wi-Fi 상태에 따라 지연·끊김이 생길 수 있어 `camera_stream` 에 재연결을 둡니다.
- 폰을 거치대에 고정한 채로 체커보드·모눈종이를 찍어 내부 보정·호모그래피를 구하고, 같은 위치·해상도에서 부품을 인식합니다. 폰이나 작업면이 움직이면 다시 보정합니다.
- MVTec 이미지는 계속 검출 로직 개발·검증용입니다. 사진 폴더·동영상 파일 입력(`image_dir`)은 폰 영상을 못 받을 때의 대체 입력과 같은 장면을 반복 재현하는 테스트용으로 남깁니다.
- 남은 일: 폰 거치대 준비, 3명 모두 WSL 에서 폰 영상 수신 확인, `camera_stream.FrameSource` 구현.
- 검증 상태: **미확인** (폰 영상 수신 아직 시험 안 함).

## 8. 팀원 2 (성민) — 1주차 시뮬레이터 작업 기록 (2026-11-13)

### 담당 파일과 기록 위치

- 실제 구현·설정: `src/ur5e_pick_place/launch/sim.launch.py`, `worlds/workcell.sdf`, `urdf/ur5e_vacuum.urdf.xacro`, `config/sim.yaml`.
- 목표·역할 분담: `docs/PROJECT.md` 2.3절. 환경·실행 절차: `docs/SETUP.md`, `docs/WEEK1.md`.
- 오류·조치·검증 결과: 이 문서. `build/`, `install/`, `log/`는 생성 결과이므로 원본 수정 위치가 아님.

### 확인한 결과

| 항목 | 결과 및 근거 |
|---|---|
| 기본 UR5e 예제 | `ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e` 실행 |
| 기본 예제 경로 계획 | 처음에는 약 5초 후 OMPL 실패. 목표를 조정한 뒤 `Motion plan was computed successfully` 출력, 계획 시간 약 0.04055초 |
| 기본 예제 실행 | Execute 후 Gazebo에서 팔이 움직였다고 사용자 확인. 기본 예제 계획·실행 확인 완료 |
| 프로젝트 robot_description 오류 | UR 런치가 Xacro 출력을 YAML로 해석할 때 주석의 `1주차 확인:`에서 파싱 실패 재현. 콜론과 공백을 없애는 주석 수정 |
| 수정 후 파라미터 검증 | 실제 Xacro 출력의 ROS launch 파라미터 변환 및 XML 파싱 통과. 흡착 패드와 DetachableJoint 슬롯 3개 존재 확인 |
| 프로젝트 반복 경고 | 아직 스폰되지 않은 obj_0~2의 모델·link를 DetachableJoint가 찾으면서 경고 출력. `suppress_child_warning=true` 추가하여 해당 경고를 생략하도록 설정 |
| 빌드 | `colcon build --symlink-install --packages-select ur5e_pick_place` 성공 |

경고 생략은 물체 스폰·흡착 구현을 완료했다는 뜻이 아니다. 전체 런치 재실행 후 경고 해소와 동작은 별도 확인해야 한다.

### 다음 확인 항목

- [ ] 프로젝트 Gazebo에서 UR5e·흰색 매트·상자 2개·흡착 패드가 모두 보이는지 확인.
- [ ] 프로젝트 작업 셀에서 Plan → Execute 후 Gazebo 팔 이동 확인 (기본 예제와 구분).
- [ ] `sim.yaml`과 `workcell.sdf` 및 실행 중 Pose 비교. 작업 평면 원점 (0.30, -0.15, 0.0)m, 매트 중심 (0.4485, -0.045, 0.0005)m, 크기 0.297×0.210m. 상자 중심은 screw (0, 0.45), nut (0, -0.45)m.
- [ ] URDF·SRDF 로봇 이름 불일치 메시지의 재발 여부 확인. 기본 예제의 이동 성공만으로 이 문제가 해결됐다고 판정하지 않음.
- [ ] 화면·로그 증거와 확인 결과를 이 절에 추가하고 팀원 리뷰 후 GitHub에 공유.

### 재개 명령

기존 시뮬레이터는 해당 터미널에서 Ctrl+C로 종료하고, 새 실행을 하나만 띄운다.

```bash
source ~/vision-arm/env.sh
ros2 launch ur5e_pick_place sim.launch.py
```

자동 물체 생성·실제 attach/detach·상자의 MoveIt 충돌 회피·자동 분류 사이클은 아직 미검증이며 1주차 화면·수동 이동 검증과 구분한다.

### 팀원 2 후속 자동 검증 (2026-11-13)

최신 코드 재빌드와 35개 테스트를 이 PC에서 통과했다. 실행 중 장면에서 매트·두 상자 좌표와 로봇·흡착 패드를 확인했고 프로젝트 지정 이동 및 흡착·해제는 재시도에서 성공했다. 최초 이동 계획 실패와 GUI·실측 자료 미검증은 구분해서 남겼다. 자세한 결과·재현 명령·미완료 항목은 [TEAM2_PROGRESS.md](TEAM2_PROGRESS.md)를 따른다. 이전 체크리스트의 장면·좌표 항목은 자동 확인했으며 GUI 직접 확인은 여전히 남아 있다.

## 9. 캘리브레이션을 ArUco 기반으로 변경 (2026-11-16, 팀원 1 동경)

체커보드·모눈종이 방식을 ChArUco 보드·ArUco 작업 시트로 바꿉니다. 출력 파일(`intrinsics.npz`, `homography.npz`)의 형식은 그대로라 `calibrator.py` 와 그 뒤 코드는 바뀌지 않습니다.

| 단계 | 기존 | 변경 | 이유 |
|---|---|---|---|
| 렌즈 왜곡 보정 (과제 필수) | 체커보드 15~20장 | **ChArUco 보드** 15~20장 (`data/calib/charuco_board_A4.pdf`) | 보드 일부가 가려지거나 화면 밖이어도 보이는 코너만으로 계산된다 → 왜곡이 큰 화면 가장자리까지 덮어 찍기 쉽다. 체커보드는 전체가 보여야 인식됨 |
| px → mm | 모눈종이 + 교차점 px 좌표를 손으로 찍은 CSV | **작업 시트**(`data/calib/work_sheet_A4.pdf`) 네 귀퉁이 ArUco 마커 (ID 40~43) | 꼭짓점 16개를 서브픽셀로 자동 검출. 손 클릭 오차가 없고, 폰이 움직여도 명령 한 번으로 재계산 |
| 좌표계 원점·축 | 종이에 손으로 표시 | 시트 왼쪽 아래 모서리, 인쇄된 화살표 | 마커 mm 위치가 인쇄로 정해져 실측이 필요 없다. Gazebo `work_mat`(A4) 과 원점·크기가 같다 |

- 왜곡 보정을 빼고 마커 4개만 쓰는 안도 검토했지만, 과제 필수 항목이고 마커가 왜곡이 가장 큰 귀퉁이에 놓이므로 **두 단계 모두 유지**한다.
- 기존 체커보드(`--board chessboard`)와 CSV(`--points`) 방식도 스크립트 옵션으로 남겨 두었다.
- OpenCV 4.6(apt) 의 옛 aruco API 와 4.7+ 새 API 를 모두 지원한다. ChArUco 행 수를 홀수(5)로 둔 것은 OpenCV 4.8 의 짝수 행 보드 무늬 변경을 피하기 위해서다.
- 사용법: `PYTHONPATH=src/vision_pick_place python scripts/make_calib_sheets.py` 로 PDF 생성 → A4 가로 100% 인쇄 → `scripts/calibrate_camera.py --images data/calib/charuco` → `scripts/compute_homography.py --aruco --image data/calib/sheet.png`.
- 검증 상태: 합성 이미지 테스트 7개 추가 (`test_aruco_calib.py`, 전체 42개 통과). PDF 를 렌더링한 이미지에서 마커 4개·보드 코너 24개 검출 확인. **실제 인쇄·폰 촬영은 미실시.**
- 후속 과제: 작업 시트 마커(검은 사각형)가 카메라에 보이므로 `vision_detector` 가 마커를 부품으로 검출할 수 있다. 호모그래피로 마커 영역을 마스크하거나 검출 ROI 를 시트 안쪽으로 제한해야 한다 (검출 담당과 협의).

## 10. 캘리브레이션 실측 결과와 왜곡 모델 선택 (2026-11-17, 팀원 1 동경)

9절 방식으로 실제 폰 촬영을 해 보니 세 가지를 바꿨다. 출력 파일 형식은 그대로다.

| 항목 | 9절 계획 | 실제 | 이유 |
|---|---|---|---|
| ChArUco 보드 | A4 인쇄 + 단단한 판 | **모니터 전체화면** (`data/calib/charuco_screen_1920x1080.png`) | 내부 파라미터는 보드 크기와 무관하다. 모니터는 휘지 않고 준비물이 필요 없다 |
| 왜곡 모델 | OpenCV 기본 5계수 (k1 k2 p1 p2 k3) | **왜곡 0 고정** (`--dist-model none`) | 아래 비교표. 폰이 이미 왜곡을 펴서 보낸다 |
| 해상도 | (정하지 않음) | 스트림과 같은 **가로 1280×720** | 보정 사진과 검출 영상의 크기가 다르면 `vision_pipeline` 이 거부한다 |

**왜곡 모델 비교.** 같은 보드 사진 18장으로 모델만 바꿔 계산하고, 작업 시트 사진(마커가 사진 끝에 붙은 1장)에서 "한 점씩 빼고 잰 오차" 를 쟀다.

| 왜곡 모델 | 보드 RMS (px) | 작업 시트 오차 (mm) |
|---|---|---|
| 5계수 (OpenCV 기본) | 0.17 | 11.96 |
| k1 k2 | 0.17 | 8.70 |
| k1 하나 | 0.35 | 1.64 |
| **0 고정** | 0.56 | **0.71** |

- 보드 사진이 화면 가운데에 몰려 있어서, 계수가 많은 모델은 데이터가 없는 사진 구석에서 크게 튀었다 (점이 30~60 px 이동, 맨 구석은 계산 실패).
- 계수를 줄일수록 시트 오차가 줄었고 0 일 때 가장 좋았다. 보드 RMS 도 0.56 px 로 1 px 미만이다 → 아이폰 카메라 앱이 렌즈 왜곡을 이미 보정해서 내보내는 것으로 판단했다.
- 마커를 사진 끝에서 떨어뜨려 다시 찍은 사진들에서는 5계수도 0.34~0.50 mm 였지만, 0 고정이 3장 중 2장에서 더 좋았다 (0.26~0.49 mm).
- 9절의 "왜곡 보정은 과제 필수" 는 지켰다: 보정을 **실제로 수행하고 모델을 비교한 결과** 0 을 골랐다. 5계수 결과는 `data/calib/archive/intrinsics_full_model.npz` 로 남겨 비교·발표에 쓴다.
- 구석까지 덮도록 보드 사진을 더 찍으면 5계수 모델이 나아질 수 있다. 다른 폰·카메라로 바꾸면 이 비교를 다시 한다.

**최종 결과** (가로 1280×720, 거치대 고정, [WEEK1.md](WEEK1.md) 팀원 1 기록):

| 파일 | 값 |
|---|---|
| `intrinsics.npz` | 18장, RMS 0.56 px, fx 883.4, fy 881.9, cx 639.6, cy 356.8, 왜곡 0 |
| `homography.npz` | 꼭짓점 16개, RMS 0.21 mm, 한 점씩 빼고 잰 오차 0.26 mm. 1 mm ≈ 1.66 px |

- 추가한 옵션: `calibrate_camera.py --dist-model {full,k1,none}`, `--rotate {none,cw,ccw,180}`. 기본값은 그대로(`full`, `none`)라 기존 명령은 결과가 같다.
- `--rotate` 방향(cw/ccw)은 왜곡 0 모델에서 mm 결과에 영향이 없다 (중심점이 화면 가운데 근처라 두 방향 결과가 거의 같고, 왜곡 0 이면 px→mm 는 호모그래피만 쓴다).
- 운용 주의: 거치대·시트를 움직이면 시트 1장을 다시 찍고 `compute_homography.py` 만 다시 돌린다. 1단계는 다시 할 필요 없다.
- 남은 일: 실제 나사·너트로 검출·크기(1 mm ≈ 1.66 px 에서 `min_area_px` 500 이 작은 부품을 버리는지) 확인.

## 11. 검출·분류 모델 비교 계획 (2026-11-19, 팀원 1 동경)

PROJECT.md 9장에 "4주 이후 선택" 으로 있던 방식 비교(규칙 → k-NN/SVM → OBB)를 **3·4주차로 앞당겨** 진행한다. 담당은 팀원 1 (동경) 혼자, 결과는 검출 담당(팀원 2)과 공유한다.

**왜 하나.**

- 지금 검출은 학습 모델 없이 OpenCV 규칙만 쓴다 (`vision_detector.py`: Blur → 이진화 → 윤곽선 → `minAreaRect` → 장축/단축 비 ≤ 1.5 면 너트).
- MVTec test 60장 기준 gray·임계값 100 은 정밀도 42.8%·재현율 63.4% (F1 51.1%), MVTec 전용 `metal_hsv` 는 77.6%·83.4% (F1 80.4%). 붙어 있는 부품이 하나로 합쳐지고 나무 무늬·그림자가 오검출된다 (WEEK2.md 6절).
- 현업에서 딥러닝을 무조건 쓰지는 않는다. 조명·배경·카메라가 고정된 공장 라인은 규칙 기반 비전(Cognex, HALCON 등)이 아직 많다: 빠르고, 결과가 항상 같고, 판정 이유를 설명할 수 있고, 학습 데이터가 필요 없다. 조명·배경이 바뀌거나 부품이 겹치거나 종류가 많으면 딥러닝(YOLO 류 검출, 분할)을 쓴다. 실제로는 **딥러닝으로 찾고 위치·각도·치수는 고전 기하로 정밀하게** 계산하는 혼합 구성이 흔하다.
- 그래서 하나를 정해 놓고 쓰지 않고, **같은 데이터·같은 기준으로 비교한 표**를 근거로 고른다.

**비교할 방식.**

| 방식 | 내용 | 비교 범위 |
|---|---|---|
| A. 규칙 (지금, 기준선) | 이진화(gray·Otsu / `metal_hsv`) → 윤곽선 → 길이 비 | 검출 + 분류 |
| B. 형상 특징 + 고전 ML | 윤곽선마다 면적·장축 길이·길이 비·볼록도·구멍 수·Hu 모멘트 → k-NN / SVM / 랜덤포레스트 (scikit-learn) | 분류 (검출은 A 와 같음) |
| C. CNN 분류기 | 부품 영역을 잘라 작은 CNN(예: ResNet18 전이학습) 으로 분류 | 분류 |
| D. YOLOv8-OBB | 회전 박스 검출 + 분류를 한 번에 학습 (가장 작은 `yolov8n-obb`) | 검출 + 분류 + 각도 (A 와 직접 비교) |

**후보를 이렇게 고른 이유.**

네 방식은 아무거나 고른 것이 아니라, **한 단계에 한 가지씩만 바꾸는 사다리**로 골랐다. 그래야 성능이 오르거나 내렸을 때 무엇 때문인지 알 수 있다.

| 단계 | 바꾸는 것 | 그대로 두는 것 | 답하려는 질문 |
|---|---|---|---|
| A → B | 분류를 규칙에서 **손으로 만든 특징 + 학습**으로 | 검출 (A 와 같은 박스) | 틀리는 원인이 분류인가? |
| B → C | 특징을 사람이 정하지 않고 **사진에서 직접 학습** | 검출 | 손으로 만든 특징으로 부족한가? |
| A·B·C → D | **검출까지 학습** | 같은 test·같은 채점 | 붙어 있는 부품을 나눠 찾을 수 있나? (규칙의 구조적 한계, 아래 분석) |

| 방식 | 고른 모델 | 근거 | 후보였지만 뺀 것 |
|---|---|---|---|
| A | 지금 `vision_detector` (gray / `metal_hsv`) | 실제 `vision_node` 가 쓰는 방식. 기준선이 없으면 "얼마나 좋아졌나" 를 말할 수 없다 | — |
| B | **k-NN, SVM(RBF), 랜덤포레스트** 셋 다 | 원리가 서로 다른 대표 분류기 셋: k-NN = 거리 (학습 없는 가장 단순한 ML 기준), SVM = 경계 여백 (샘플 수천 개·특징 수십 개인 작은 표 데이터에 강함), 랜덤포레스트 = 나무 앙상블 (특징 단위가 달라도 되고 **특징 중요도로 설명 가능**). 셋 다 scikit-learn 에 있고 CPU 로 수 초 | 그래디언트 부스팅(XGBoost 등): 작은 데이터에서 랜덤포레스트와 비슷하고 패키지만 늘어남. 로지스틱 회귀: 직선 경계라 13종에 약함. MLP: SVM 과 역할이 겹치고 조정할 값이 많음 |
| C | **ResNet18 (ImageNet 사전학습) + 크기 2개** | 잘라 낸 부품이 약 3천 장뿐이라 처음부터 학습하기엔 적다 → 사전학습 모델을 미세조정. ResNet18 은 표준 백본 중 가장 작은 편(약 1,100만 파라미터)이라 CPU 로 미세조정이 된다. 사진을 같은 크기로 줄이면 길이 정보가 사라져서 실제 길이(log 장축·단축)를 따로 넣었다 | 직접 만든 작은 CNN: 데이터가 적어 사전학습보다 불리. ResNet50·EfficientNet·ViT: CPU 학습이 몇 배 느리고, ViT 는 데이터가 더 많아야 함. MobileNet: 더 빠르지만 미세조정이 덜 안정적이라 첫 비교는 표준인 ResNet18 로 |
| D | **YOLOv8n-OBB** | ① **회전 박스(OBB)** 여야 한다: 우리 출력에 각도가 필요하고, MVTec 정답도 회전 박스다. 일반 축 정렬 박스는 비스듬한 나사끼리 박스가 크게 겹치고 각도를 주지 않는다. ② Ultralytics 가 OBB 를 공식 지원하고 사전학습 가중치(DOTA)가 있어 `pip` 하나로 학습·추론이 된다. ③ **n(가장 작은 크기)**: 약 300만 파라미터·8.3 GFLOPs 라 CPU 학습이 약 2시간, 1 epoch 시험 때 추론 약 75 ms/장 (1024 입력) 으로 10 Hz(100 ms) 안에 들어온다 | Mask R-CNN 등 인스턴스 분할: 더 무겁고, MVTec 에는 박스 정답만 있고 마스크 정답이 없다. 회전 Faster R-CNN (MMRotate): 설치가 복잡하고 GPU 위주. YOLO11-OBB: 최신이고 성능이 비슷하거나 조금 나은 것으로 알려져 있으나, 첫 비교는 자료가 많은 v8 로 하고 필요하면 바꿔 본다. YOLOv8 s·m 크기: CPU 학습이 몇 배 오래 걸림 |

- 참고: MVTec Screws 를 공개한 논문(Ulrich 등, 2019, DOI 10.1515/teme-2019-0076)도 이 데이터로 **모양 기반 매칭(고전)과 딥러닝 검출을 비교**했다. 우리 비교도 같은 질문이다. 논문의 모양 기반 매칭은 상용 HALCON 기능이라 쓰지 못해서, 고전 쪽은 우리 규칙(A)과 특징+ML(B)로 대신한다.

**평가 기준 (모두 같은 test 세트).**

- 데이터: MVTec Screws 의 기존 train / val / test 분할(269 / 55 / 60장). 설정·모델 선택은 val 로만 하고 test 는 마지막에 한 번 잰다 (2주차 검출 평가와 같은 규칙).
- 지표: 검출 정밀도·재현율·F1 (회전 IoU ≥ 0.5, `evaluate_detector.py` 의 대응 방식 재사용), 분류 정확도·혼동 행렬, 중심 오차(px), 장축 각도 오차(°), **CPU 추론 시간**(이미지 1장 / 부품 1개, ms), 학습 시간.
- 분류는 MVTec 13종 전체와, 프로젝트 범위인 나사/너트 2종 두 가지로 잰다.
- 4주차: 자체 촬영 사진(실측 정답 30+ 자세, PROJECT.md 6.1)에서 상위 방식을 다시 잰다. MVTec 성능이 우리 작업 시트·조명에서 그대로 나온다고 가정하지 않는다.

**환경 제약: GPU 없음.**

- 팀 PC 는 모두 Intel 내장 그래픽이라 CUDA 학습을 못 한다. WSL 메모리 약 9.7 GiB.
- B 는 수 초, C 는 수 분~수십 분, D(`yolov8n-obb`, 384장)는 CPU 로 약 1~3시간으로 예상 (미측정 → **실측: C 약 12분, D 약 2.1시간, 2026-11-19**). 오래 걸리면 Google Colab 무료 GPU 로 **학습만** 하고 가중치 파일을 받아 CPU 로 추론한다.
- 추론은 CPU 로 충분할 것으로 본다: 검출 목표 10 Hz, 검출→송신 P95 ≤ 300 ms (PROJECT.md 7장). 실제 추론 시간을 비교표에 같이 적어 확인한다.
- 학습 패키지(torch, ultralytics, scikit-learn)는 pip 판 OpenCV·NumPy 를 끌어와 ROS 용 `.venv` 와 충돌할 수 있어 **별도 venv(`.venv-ml`)** 에 설치한다 (SETUP.md Step 8).

**결정 규칙.**

- 비교표의 정확도와 CPU 추론 시간을 같이 보고 고른다. 규칙(A)과 차이가 작으면 단순하고 설명 가능한 A 를 유지한다.
- 학습 모델을 `vision_node` 에 넣게 되면 패킷 형식(`part_type`, `x_mm`, `y_mm`, `angle_deg`)과 px→mm·각도 계산(`calibrator.py`)은 그대로 두고 검출 부분만 바꾼다. 넣을지는 팀이 같이 정한다.
- 결과 표는 이 절과 PROJECT.md 7장(선택 KPI)에 적는다.

| 시점 | 할 일 | 끝났다는 기준 |
|---|---|---|
| 3주차 (GUI 와 병행) | 공통 평가 스크립트, MVTec 으로 A·B·C·D 비교 | MVTec test 비교표 (정확도·각도·CPU 추론 시간) |
| 4주차 (KPI 와 병행) | 자체 촬영 사진으로 상위 방식 재평가, 채택 여부 결정 | 자체 사진 비교표 + 채택 결정 기록 |

**MVTec test 결과 (2026-11-19, 3주차 항목 완료).** 정답 661개, 회전 IoU ≥ 0.5. 설정·신뢰도 기준은 val 에서만 골랐고 test 는 한 번만 썼다. 과정·그래프는 `notebooks/model_comparison.ipynb`.

| 방식 | 검출 F1 | 분류 13종 (정답 박스) | 분류 2종 | 종합 F1 (2종) | 중심 오차 | 각도 오차 | CPU 시간/장 |
|---|---|---|---|---|---|---|---|
| A-gray | 0.511 | — | 89.0% | 0.455 | 14.6 px | 5.46° | 6 ms |
| A-hsv (지금 `vision_node`) | 0.804 | — | 92.9% | 0.747 | 9.5 px | 5.20° | 23 ms |
| A-hsv + B-SVM | 0.804 | 99.2% | 100% | 0.804 | 9.5 px | 5.20° | 39 ms |
| A-hsv + B-RF | 0.804 | 99.5% | 100% | 0.804 | 9.5 px | 5.20° | 125 ms |
| A-hsv + C-CNN (ResNet18) | 0.804 | 99.4% | 98.9% | 0.795 | 9.5 px | 5.20° | 117 ms |
| **D-YOLOv8n-OBB** | **0.989** | 97.9% (검출된 것) | 100% | **0.989** | **4.4 px** | **1.12°** | 98 ms |

- **병목은 검출이다.** 정답 박스로 분류만 하면 B·C 모두 13종 99% 이상, 나사/너트 100%. 규칙의 분류 오류는 거의 `type_013`(짧은 나사)을 너트로 본 것이고, B 를 붙이면 종합 F1 0.747 → 0.804 가 된다. 하지만 검출 F1 0.804 에서 막힌다.
- **YOLO 는 붙은 부품을 나눈다.** 놓친 정답 중 "다른 부품과 합쳐짐" 62 → 2, 오검출 중 "배경" 87 → 0. 각도 오차 5.2° → 1.1°.
- **C(CNN)는 B 보다 나은 점이 없다.** 정확도는 같고 학습 12분·추론 약 3배. 규칙 박스로 자르면 오히려 98.4%.
- **YOLO 의 13종 오류는 크기만 다른 너트끼리**(007↔010 등 8개)라 나사/너트 구분에는 영향이 없다.
- 학습 시간 (CPU): B 수 초, C 12 epoch × 62초 ≈ 12분, D 40 epoch ≈ 2.1시간. Colab 은 필요 없었다.
- **해석의 한계.** MVTec 은 학습·시험이 같은 카메라·배경이라 YOLO 에 유리하다. 우리 환경(흰 시트, 휴대폰, 약 1.66 px/mm)에서 YOLO 를 쓰려면 자체 사진 라벨로 재학습이 필요하다. 부품이 떨어져 놓인 사진에서는 규칙도 거의 완벽했다.
- **잠정 결론.** MVTec 기준 최고는 D(차이가 커서 "차이가 작으면 A 유지" 에 해당하지 않음). 분류만 고친다면 B-SVM 이 가장 효율적. `vision_node` 적용은 4주차 자체 사진 재평가 뒤 셋이 정한다. 부품을 떨어뜨려 놓는 지금 시나리오면 A-hsv + B-SVM, 붙은 부품까지 다뤄야 하면 YOLO 재학습.

