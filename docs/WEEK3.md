# 3주차 팀원 2 — 팀 간 통신 연동

2026-11-16, `/home/user1/vision-arm`에서 구현·검증. 로컬 수정이며 아직 커밋·푸시하지 않았다.

## 연결된 흐름

```text
영상 입력(팀원 3) → 검출(팀원 2)·보정(팀원 1)
  → /vision/detections → robot_bridge → TCP → robot_server
      ← /robot/log ← ACK                  → /robot/targets → pick_place_node
      → /robot/log는 팀원 1 GUI가 추후 구독할 연결 지점
```

- 기존 검출·보정·vision_node의 패킷 형식을 유지했다. `seq`, `t_capture`, `calib_ok`, `objects`를 TCP와 ROS에서 그대로 전달한다.
- 서버 기본 실행은 ROS를 import하지 않는다. `--publish-ros`에서만 `/robot/targets`를 발행한다.
- 서버는 NDJSON 한 줄 단위 수신, 1MiB 제한, 패킷 검사, ACK 및 수신 CSV를 구현했다. 여러 클라이언트를 처리하며 잘못된 한 줄 뒤의 정상 패킷도 처리한다.
- 브리지는 ACK 번호·상태·제한 시간을 확인한다. 실패하면 연결을 닫고 다음 프레임에서 재연결한다. 수신 여부가 불명확한 패킷을 자동 재전송하지 않는다. 중복 제거·영구 exactly-once 보장은 없다.
- 브리지 구독 대기열은 1개이며, ACK 대기 동안 과거 프레임이 누적되는 것을 줄인다. 모든 카메라 프레임의 전달 보장은 없으므로 송신 성공률의 분모는 기록된 송신 시도이다.
- `/robot/log`와 `logs/comm_*.csv`: seq, sent, acked, latency_ms, error. sent는 전송 시도이며 wire 전달 증거가 아니다. latency는 ACK 성공 시 프레임 t_capture부터 계산한다.
- `logs/robot_rx_*.csv`: t_received, seq, status, error, packet. 전체 물체 목록은 packet JSON 열에 보관한다. 미구현 `evaluate_kpi.py`와의 평탄화·실측 매칭은 후속 작업이다.
- 사용자 요청으로 GUI 두 파일은 수정 전 상태로 원복했다. 다른 팀원 담당 코드는 수정하지 않는다. GUI 구현·표시는 해당 담당자의 후속 작업이다.

## 실행

기존 동일 포트 서버·파이프라인은 먼저 종료한다. 프로젝트 루트에서 실행한다.

```bash
source ~/vision-arm/env.sh
colcon build --symlink-install
source install/setup.bash
# 서버 + 브리지 + 기존 보유 이미지 비전 노드 (GUI 실행 안 함)
ros2 launch vision_pick_place integration.launch.py
```

카메라 없이 기본 MVTec 이미지 폴더로 실행한다. `params_mvtec.yaml`은 미보정 입력이므로 calib_ok=false와 null mm 좌표가 정상이다.

실제 스마트폰·보정 설정으로 바꿀 때:

```bash
ros2 launch vision_pick_place integration.launch.py \
  vision_params:=$PWD/src/vision_pick_place/config/params.yaml
```

동일 서버에 `pipeline.launch.py`를 추가로 띄우면 비전·브리지·GUI가 중복된다. 둘 중 하나만 사용한다.
통합 런치는 GUI를 실행하지 않는다. 다른 TCP 포트는 `port:=19084`를 전달한다.

별도 터미널:

```bash
source ~/vision-arm/env.sh
ros2 topic echo /robot/log
# 또는
ros2 topic echo /robot/targets
```

Gazebo 연결은 별도 터미널에서 `ros2 launch ur5e_pick_place sim.launch.py` 실행.
현재 pick_place_node는 `/robot/targets`를 수신·저장하지만 `/sim/run_cycle` 전체 집기 사이클은 여전히 미구현이다.
통신 ACK와 타깃 수신은 실제 로봇 집기 완료가 아니다. 미보정 데이터를 실제 좌표로 사용하면 안 된다.

## 실제 검증

- 전체 빌드 성공, 65개 테스트 통과 (실패·오류·skipped 0).
- 새 TCP 테스트: 분할/합쳐진 패킷, 잘못된 JSON 뒤 복구, 100회 ACK, ACK 번호 불일치 후 재연결, 타임아웃, 보정 유효성 검사.
- ROS_DOMAIN_ID=84와 TCP 19084로 기존 세션과 분리해 통합 런치 실행.
- 실제 MVTec 비전 → ROS → TCP → ACK → ROS targets에서 동일 패킷 6개 확인, TCP 오류 없음.
- GUI 수정본의 Qt 시험은 원복 전 이력이며 현재 코드의 검증 결과에서 제외한다. 검증 도구는 GUI 없이 ROS 토픽·ACK·타깃 동일성만 확인하도록 변경했다.
- Gazebo를 움직이는 명령은 보내지 않았다. 스마트폰 실시간·실제 보정·실제 GUI 화면은 이번 시험 범위 밖이다.

재현: 통합 런치를 실행한 상태에서 다른 터미널에 다음을 실행한다. 두 터미널 ROS_DOMAIN_ID는 같아야 한다.

```bash
source ~/vision-arm/env.sh
python scripts/check_team_integration.py
```

결과는 `logs/team_integration_check.json`에 저장되며 logs는 git에서 제외된다.

## 팀원별 연결 지점

| 담당 | 연결 방법 |
|---|---|
| 팀원 1 보정·GUI | 보정 결과를 기존 vision_node에서 사용. GUI는 /vision/debug_image, /vision/detections, /robot/log, /sim/pick_log 구독 |
| 팀원 2 검출·통신 | 기존 process_frame 출력 유지, TCP 수신 확인과 CSV 증거 제공 |
| 팀원 3 영상·스폰 | 영상 입력을 vision_node로 제공. 스폰·흡착 구현은 /robot/targets를 소비하는 로봇 사이클과 연동 |

후속: 실제 보정 패킷 현장 확인, GUI 상세 보정 상태, 서버 중복 처리 정책, 타깃→자동 스폰·집기 사이클, CSV→실측 KPI 집계.

## 다른 팀원 코드를 수정하지 않는 연결 방법

다른 팀원의 GUI·보정·영상 입력·로봇 제어 원본은 HEAD와 동일하게 유지한다.
팀원 2만 실행하려면 새 전용 런치를 사용한다. 이는 서버·브리지 두 노드만 실행하며 비전·GUI·Gazebo를 실행하지 않는다.

```bash
source ~/vision-arm/env.sh
colcon build --symlink-install --packages-select vision_pick_place
source install/setup.bash
ros2 launch vision_pick_place tcp.launch.py
```

다른 팀원은 기존 코드와 실행 명령을 사용한다. 두 PC로 나눠 실행하는 네트워크 배포는 별도 ROS discovery 설정이 필요하며 이번에는 같은 WSL 환경에서 시험했다.

| 연결 지점 | ROS 형식 | 유지하는 규약 |
|---|---|---|
| 입력 /vision/detections | std_msgs/msg/String | 기존 JSON을 구독. 검출·보정 계산을 재작성하지 않음 |
| 출력 /robot/targets | std_msgs/msg/String | 입력 패킷 전체 그대로 전달. mm·각도·null·seq 변경 없음 |
| 출력 /robot/log | std_msgs/msg/String | seq, sent, acked, latency_ms, error. GUI 담당자가 구독 가능 |

1. 팀원 3의 기존 vision_node 실행 → 위 TCP 런치가 자동으로 구독한다.
2. 팀원 1의 GUI가 준비되면 기존 규약의 /robot/log 등을 구독한다. 현재 GUI가 뼈대이면 그 구현은 팀원 1에게 맡긴다.
3. 기존 sim.launch.py의 pick_place_node가 /robot/targets를 수신한다. 그 안의 스폰·자동 집기 구현은 해당 담당자가 진행한다.

주의: tcp.launch.py와 integration.launch.py는 같은 서버·브리지를 포함하므로 동시에 실행하지 않는다.
기존 pipeline.launch.py도 브리지를 포함하므로 전용 런치와 중복 실행하지 않는다.
보정된 나사·너트 패킷의 음수 좌표, 179.5° 각도, 너트 null 각도와 px 좌표가 변형 없이 전달되는 TCP 시험도 추가했다.

최종 재검증: 팀원 2 tcp.launch.py와 수정하지 않은 기존 vision_node를 분리 실행해 동일 패킷 5개 전달·ACK·영상 토픽 수신 성공, TCP 오류 0. GUI·보정·영상·로봇 담당 원본의 HEAD 대비 차이 없음 확인.
