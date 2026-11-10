# 설계 결정 기록

받은 "UR5e 기반 ROS2 & Gazebo 픽앤플레이스 3인 역할 분담표"를 이 저장소에 어떻게 반영했는지, 무엇을 바꿨고 왜 바꿨는지 남깁니다.
각 결정은 **검증 상태**를 같이 적습니다. "문서 확인" 은 공식 문서를 읽고 확인한 것, "미확인" 은 아직 직접 돌려 보지 않은 것입니다.
(작성 및 환경 재점검: 2026-11-10. ROS2 Jazzy와 시뮬레이터 패키지 설치 완료. 실제 구동에서 발견한 설치 차단 항목은 아래에 기록합니다.)

### 현재 PC 실행 점검 (2026-11-10)

- Python·ROS import, pip 의존성 검사 정상. 테스트 7개 중 3개 통과, 4개 미구현으로 skipped.
- WSL 설정: `C:\Users\user1\.wslconfig`에 memory=10GB, swap=8GB 작성. 현재 실행 중인 WSL은 7.6 GiB이며 재시작 후 적용 확인 필요.
- 이전에 누락된 `ur_robot_driver`는 Jazzy 3.9.0 설치 확인 완료. sudo 인증 성공, rosdep check와 pip check 통과. 실행 의존성을 package.xml에 추가함.
- UR 기본 예제 재실행에서 MoveIt 초기화(`You can start planning now!`) 및 joint_state_broadcaster / scaled_joint_trajectory_controller 활성화 확인. URDF·SRDF 로봇 이름 불일치와 선택적 인식/Octomap 기능 오류가 남아 있으며, 실제 화면·Plan & Execute·프로젝트 작업 셀은 미검증.
- `/dev/video*` 없음. 현재 도구 환경은 Windows PowerShell 실행이 `Exec format error`로 실패하므로 usbipd 설치 폴더는 현재 존재함. 버전·서비스·USB 장치 확인은 Windows에서 실행해야 함.
- 다음 절차: `WEEK1.md`의 현재 PC 남은 작업 참고. WSL 재시작은 현재 세션을 종료하므로 사용자가 Windows에서 수행.

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
| `ur_sim_control.launch.py` 가 `description_file`, `world_file`, `gazebo_gui`, `launch_rviz` 를 받는지 (Jazzy 판) | `--show-args` (SETUP.md 5-3) | `launch/sim.launch.py` | | |
| `ur_moveit.launch.py` 가 `ur_type`, `use_sim_time`, `launch_rviz` 를 받는지 | `--show-args` | `launch/sim.launch.py` | | |
| `ur_simulation_gz/urdf/ur_gz.urdf.xacro` 경로 | `ls` (SETUP.md 5-3) | `urdf/ur5e_vacuum.urdf.xacro` | | |
| 계획 그룹 이름 `ur_manipulator` | RViz MotionPlanning 패널 | `pick_place_node.py` | | |
| 고정 관절로 붙은 `tool0` 이 SDF 변환 뒤 `wrist_3_link` 로 합쳐지는지 | `gz model -m ur -l` 로 링크 목록 | `urdf/ur5e_vacuum.urdf.xacro` | | |
| DetachableJoint 가 **나중에 스폰한** 모델을 잡는지, 스폰 즉시 붙는지 | 확인 ② (손으로 스폰 → detach → attach) | `pick_place_node.py`, `object_spawner.py` | | |
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

## 6. 실시간 웹캠 연결 제외 (2026-11-10)

실시간 USB 웹캠 연결·usbipd 설정만 제외합니다. 실제 모눈종이 촬영과 카메라 좌표계(px) → 로봇 작업 좌표계(mm) 변환 계획은 유지합니다.

- 보유 MVTec 사진은 물체 검출·중심점·장축 각도 계산 로직을 개발·검증하는 데이터입니다.
- 실제 카메라로 촬영한 체커보드·모눈종이·부품 사진을 파일로 가져옵니다. 격자 실측 좌표로 보정하고 같은 촬영 조건의 부품 위치·방향을 mm·도로 변환합니다.
- 보유 사진에는 mm 정답이 없으므로 임의 크기를 부여한 결과를 실제 위치 KPI로 삼지 않습니다.
- 변환 결과를 Gazebo UR5e에 전달합니다. 가상 카메라 도입은 결정하지 않았습니다.
- 이미지 입력·보정 스크립트·관련 노드는 아직 구현이 필요합니다. usbipd는 설치되어 있지만 현재 작업의 필수 항목이 아닙니다.
- 기존 역할 분담표의 웹캠 언급은 이전 계획 기록이며, 현재 범위는 이 절을 따릅니다.
