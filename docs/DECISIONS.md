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
