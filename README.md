# vision-arm

로봇 픽앤플레이스용 좌표·방향 산출. 평면 물체의 X·Y·회전각을 비전으로 추정해 가상 로봇에 TCP 로 송신하고,
Gazebo 속 UR5e 가 그 결과로 부품을 집어 종류별로 분류합니다.

| 단계 | 내용 |
|---|---|
| 과제 필수 | 중심·장축 각도 추정, px → mm 캘리브레이션, TCP 송신·ACK, GUI 4화면, KPI 4종 |
| 확장 (4주) | Gazebo Harmonic + UR5e + MoveIt2 로 **나사·너트** Pick & Place 분류 |
| 최종 목표 | 같은 구조로 **여러 종류의 공구**까지 분류 |

## 문서

- [docs/PROJECT.md](docs/PROJECT.md): 프로젝트 정의, 좌표·각도 정의, 시스템 구성, 역할 분담(주차별 순환), KPI, 4주 로드맵, 최종 목표
- [docs/SETUP.md](docs/SETUP.md): 개발 환경 구축 (ROS2 Jazzy, venv, rosdep 으로 Gazebo·MoveIt, 시뮬레이터 구동 확인, 웹캠)
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
git clone https://github.com/robot-vision-arms/vision-arm.git ~/study/robot_vision
# docs/SETUP.md 의 Step 0~4 (conda 끄기, ROS2 Jazzy, .venv, rosdep, 빌드) 를 먼저 완료
source ~/study/robot_vision/env.sh
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

1주차 시작 전. 두 패키지 모두 **뼈대만** 있습니다. 핵심 함수는 `NotImplementedError` / `TODO` 상태이고,
아직 구현 안 된 함수의 테스트는 `@pytest.mark.skip` 으로 꺼 두었습니다.
KPI 는 측정 전이고, Gazebo·MoveIt 쪽 런치 인자와 흡착 동작은 1~2주차에 확인합니다 ([DECISIONS.md 4장](docs/DECISIONS.md#4-아직-확인-못-한-것-12주차에-채움)).
