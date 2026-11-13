# vision-arm

로봇 픽앤플레이스용 좌표·방향 산출. 평면 물체의 X·Y·회전각을 비전으로 추정해 가상 로봇에 TCP 로 송신하고,
Gazebo 속 UR5e 가 그 결과로 부품을 집어 종류별로 분류합니다.

| 단계 | 내용 |
|---|---|
| 과제 필수 | 중심·장축 각도 추정, px → mm 캘리브레이션, TCP 송신·ACK, GUI 4화면, KPI 4종 |
| 확장 (4주) | Gazebo Harmonic + UR5e + MoveIt2 로 **나사·너트** Pick & Place 분류 |
| 최종 목표 | 같은 구조로 **여러 종류의 공구**까지 분류 |

기본 입력은 거치대에 고정한 스마트폰의 실시간 영상입니다. 폰 IP 카메라 앱의 영상 주소를 Wi-Fi 로 받습니다 (docs/SETUP.md Step 7).
같은 폰·같은 위치로 체커보드·모눈종이를 찍어 px→mm 좌표 보정을 하고 부품을 인식합니다.
보유 MVTec 이미지는 검출 로직 개발·검증용이고, 사진 폴더·동영상 파일 입력(`image_dir`)은 폰 영상을 못 받을 때의 대체 입력입니다.
이미지 폴더·카메라 입력, 전처리와 좌표 보정 코드를 구현했습니다. 실제 폰 촬영·보정값 검증은 남아 있습니다.

## 문서

- [docs/PROJECT.md](docs/PROJECT.md): 프로젝트 정의, 좌표·각도 정의, 시스템 구성, 역할 분담(주차별 순환), KPI, 4주 로드맵, 최종 목표
- [docs/WEEK1.md](docs/WEEK1.md): 1주차 환경 설치 체크리스트 (실행 순서, 단계별 통과 기준, 결과 기록 위치)
- [docs/WEEK2.md](docs/WEEK2.md): 2주차 구현 상태, 비전·이동·흡착 실행 방법, MVTec 평가와 남은 실측 작업
- [docs/SETUP.md](docs/SETUP.md): 개발 환경 구축 (ROS2 Jazzy, venv, rosdep 으로 Gazebo·MoveIt, 시뮬레이터 구동 확인, 스마트폰 카메라)
- [docs/DECISIONS.md](docs/DECISIONS.md): 받은 역할 분담안과 비교해 바꾼 점, 이유, 검증 상태, 아직 확인 못 한 것

## 구조

```
 카메라 → vision_node → robot_bridge ──TCP──► robot_server ──(--publish-ros)──► pick_place_node → MoveIt2 → Gazebo UR5e
                │                              (가상 로봇, ACK)                    (/sim/run_cycle)
                └──────────► GUI (PySide6)
```

| 패키지 | 범위 | 내용 |
|---|---|---|
| `src/vision_pick_place` | 과제 필수 | 비전, 캘리브레이션, 패킷, TCP 브리지·서버, GUI |
| `src/ur5e_pick_place` | 확장 | Gazebo 작업 셀, UR5e + 흡착 패드, 부품 스폰, MoveIt2 Pick & Place |

## 빠른 시작

```bash
git clone https://github.com/robot-vision-arms/vision-arm.git ~/vision-arm
# docs/SETUP.md 의 Step 0~4 (conda 끄기, ROS2 Jazzy, .venv, rosdep, 빌드) 를 먼저 완료
source ~/vision-arm/env.sh
colcon build --symlink-install && source install/setup.bash

# 과제 필수 범위
ros2 run vision_pick_place robot_server            # 터미널 1
ros2 launch vision_pick_place pipeline.launch.py   # 터미널 2

# Gazebo 확장까지
ros2 launch ur5e_pick_place sim.launch.py                   # 터미널 1
ros2 run vision_pick_place robot_server --publish-ros       # 터미널 2
ros2 launch vision_pick_place pipeline.launch.py            # 터미널 3
ros2 service call /sim/run_cycle std_srvs/srv/Trigger       # 터미널 4
```

## 상태

2026-11-13 기준 영상 입력·전처리, 검출·비율 분류, 보정 파일 생성·로딩과 px→mm 변환,
ROS2 검출·디버그 영상 발행, 작업면→로봇 좌표·툴 자세 계산을 구현했습니다.
Gazebo + MoveIt 지정 좌표 이동과 늦게 스폰한 부품의 attach→이동 추종→detach·낙하 시험을 통과했습니다.
`colcon test` 결과는 35개 통과, 실패·오류·skip 0개입니다.
MVTec test split의 정밀도 42.8%·재현율 63.4%이며, 실제 폰 보정·자체 촬영 데이터와 mm KPI는 아직 검증 전입니다.
전체 집기·분류 사이클, TCP·GUI·KPI 구현은 후속 작업입니다. 실행 방법과 한계는 [WEEK2.md](docs/WEEK2.md)에 기록했습니다.
