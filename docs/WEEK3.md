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

## 3주차 보완 및 재검증 — 2026-11-16

팀원 2 파일만 수정했다. GUI·보정·영상 입력·비전 노드·검출 파이프라인·로봇 제어의 HEAD 대비 변경이 없음을 확인했다.

### 수정한 부족한 부분

1. **성공 판정 강화**: 기존에는 정상 패킷 5개만 있으면 오류가 섞여도 ok=true가 될 수 있었다. 이제 기본 100개 입력을 기준으로 ACK·타깃 동일성을 모두 요구하고, 관찰 중 ACK 실패·오류·중복 seq·좌표 변경·유효하지 않은 지연이 하나라도 있으면 실패다.
2. **성공률 분리**: ACK 성공률과 동일 타깃 전달률을 별도로 집계한다. ACK 성공은 ROS 전달이나 로봇 실행 성공을 보장하지 않는다. 누락 seq도 JSON에 기록한다.
3. **지연 측정**: 정상 ACK의 프레임 t_capture→ACK 지연 P95를 계산한다. 선형 보간 95백분위수를 사용한다. 이 값은 현재 PC의 파일 입력 시험 결과이며 다른 PC·Wi-Fi 지연을 대표하지 않는다.
4. **실패 결과 저장**: 검사 시작부터 ok=false/status=running 파일을 기록하고, 성공·실패·중단 모두 현재 결과로 원자적으로 교체한다. 새 실패 뒤 과거 PASS가 그대로 남는 문제를 해결했다. 종료 코드도 성공 0, 실패 1, Ctrl+C 130으로 구분한다. 강제 종료 시에는 running 상태가 남아 성공으로 오인하지 않는다.
5. **실제 서버 재시작 시험**: 서버 중단 시 기존 연결도 종료하도록 보완했다. 실제 ROS/TCP 시험에서 실패한 seq=100의 ACK 실패를 기록하고, 서버 재시작 뒤 새 seq=101로 연결을 복구했다. 실패 패킷 자동 재전송은 하지 않는다.
6. **재시작 직후 첫 타깃 유실 수정**: 첫 100회는 성공했지만 새 ROS 발행자의 discovery 전에 첫 타깃을 발행해 잃는 문제를 재현했다. 서버는 ACK 후 /robot/targets 구독자 연결을 최대 1초 기다린다(`--target-wait-s`). 구독자가 없으면 forward_error를 CSV에 남기며 성공적으로 전달했다고 표시하지 않는다.
7. **수신과 발행 기록 분리**: 서버 CSV status의 ok는 수신·검사 성공, forward_published는 ROS publish 호출 성공, forward_error는 발행 실패다. forward_published도 실제 수신자 소비 증거는 아니므로 종단 검사는 /robot/targets를 직접 구독해 비교한다.

### 실제 시험 결과

| 시험 | 결과 |
|---|---|
| 빌드 | 두 패키지 성공 |
| 전체 테스트 | **74개, 오류 0·실패 0·skipped 0** |
| 실제 MVTec 이미지 100프레임 | ACK **100/100**, 동일 타깃 **100/100**, 오류 0 |
| 파일 입력 프레임→ACK 지연 P95 | **16.17ms**, 100개 표본 |
| 이미지 토픽 | 수신 성공 |
| 합성 보정/미보정 패킷 100개 | ACK·동일 타깃 각 **100/100** |
| 실제 서버 재시작 | 실패 기록·다음 패킷 복구·실패 패킷 재전송 없음 확인 |
| 파이프라인 없는 검사 | 종료 코드 1 및 이전 성공 JSON이 새 실패 JSON으로 교체됨 확인 |

실제 이미지 시험은 calib_ok=false다. 합성 시험은 calibrated 형식과 null·좌표·각도가 보존되는지 검증한 것이며 실제 폰 영상·카메라 보정 정확도를 검증한 것이 아니다. 로봇 이동 명령은 보내지 않았다.

### 사용 방법 (기존 실행 유지)

TCP 런치와 비전 노드를 각각 실행한 상태에서 새 터미널:

```bash
source ~/vision-arm/env.sh
python scripts/check_team_integration.py
```

기본은 100개·제한 시간 40초다. 통과하려면 디버그 영상도 수신되어야 한다. 간단한 연결 확인만 필요하면 `--count 5`를 지정한다. 이것을 100회 시험과 혼동하지 않는다.
검사 JSON의 errors, missing_acks, missing_or_changed_targets, ack_success_rate, target_delivery_rate, latency_p95_ms를 확인한다.

### 자동 격리 시험 (카메라·Gazebo 불필요)

```bash
source ~/vision-arm/env.sh
python scripts/check_team2_roundtrip.py --domain 92
```

팀원 2 서버·브리지·시험 노드만 띄우고 종료한다. 지정 도메인에 기존 로봇 타깃 구독자가 있으면 시험을 거부한다. 다른 프로젝트가 쓰지 않는 ROS 도메인을 선택한다.
100개 시험용 패킷은 보정된 형식과 미보정 형식을 번갈아 보내며, 실제 로봇 동작은 요청하지 않는다. 이 결과에 영상 수신·실측 검증이 포함됐다고 해석하지 않는다.

증거 파일 (logs는 git 제외):

- `logs/team2_images_100_check.json`: 실제 파일 영상 100회 결과
- `logs/team2_roundtrip_check.json`: 합성 100회와 실제 서버 재시작 결과
- `logs/team2_roundtrip_*/`: 해당 시험의 comm CSV, robot_rx CSV, process.log

### 남아 있는 담당 간 확인

- 실제 스마트폰·실제 보정값의 데이터 시험: 보정/영상 담당자의 입력이 필요하다.
- GUI 로그 표시: GUI 담당자가 기존 /robot/log 규약을 구독해 확인한다. GUI 파일은 수정하지 않았다.
- 로봇의 타깃 소비·자동 집기: 로봇 담당 기능과 4주차 통합 범위다.
- ACK 중복 처리의 영구 보장 및 여러 PC 간 시계 동기화는 현재 제공하지 않는다.

기록일 기준 이번 보완은 로컬 변경이며 커밋·푸시는 별도 요청 시 진행한다.

## 시계·로그·연속 운전 보완 — 2026-11-16

팀원 2 통신 모듈과 검사 도구만 수정했다. 다른 팀원 GUI·보정·영상·비전 파이프라인·로봇 제어 파일의 HEAD 대비 차이는 없다.

### 1. 시계 변화와 지연 구분

`clock_measurement.py`가 브리지 시작 이후의 wall-clock/monotonic 차이를 관찰한다.
50ms를 초과하는 변화, 미래 capture 시각, 부정확한 샘플(두 단조 시계 측정 사이 5ms 초과)을 발견하면:

- ACK 성공 자체는 유지한다.
- `latency_valid=false`, `latency_ms=null`로 기록해 프레임→ACK P95 표본에서 제외한다.
- `raw_latency_ms`, `clock_warning`, `clock_jump_ms`에 근거를 남긴다.
- 검사 결과에서 `communication_ok`와 `latency_ok`를 따로 보여준다. 종합 `ok`는 둘을 함께 요구한다.

이는 **브리지가 관찰한 시간 구간의 보호**다. 다른 팀원 소스 코드를 수정하지 않았으므로 프레임 생성 전후의 시계 안정성이나 서로 다른 PC의 시계 동기화를 입증하지 못한다.
급격한 시계 변화가 감지되지 않았다는 것만으로 전체 지연이 절대적으로 정확하다고 단정하지 않는다.
새 `tcp_roundtrip_ms`는 단조 시계로 sendall 시작→정상 ACK까지 측정해 시계 보정 영향을 피한다. 연결 수립과 비전 처리 시간은 포함하지 않는다.

### 2. 전송 단계 로그

기존 seq/sent/acked/latency_ms/error는 유지하고 아래 필드를 추가했다.

| 필드 | 의미 |
|---|---|
| attempted | 브리지 콜백의 처리 시도 |
| validated | 패킷·크기 검사 완료 |
| connected | 소켓 연결 수립 또는 기존 소켓 사용 단계 도달. 상대 생존의 최종 증거는 ACK |
| send_completed | 로컬 sendall 반환. 상대가 수신·처리했다는 증거는 아님 |
| stage | decoding / validating / connecting / sending / waiting_ack / acked 중 도달 단계 |
| tcp_roundtrip_ms | 정상 ACK까지의 단조 시계 왕복 시간 |
| latency_valid / clock_warning | 프레임→ACK 지연의 관찰 범위 내 유효성·경고 |

실패 시 stage와 error를 함께 보면 데이터 검사·연결·쓰기·응답 중 어디서 실패했는지 구분할 수 있다.
호환성을 위해 기존 sent는 전송 시도라는 의미를 유지한다. 신규 집계에서는 send_completed와 acked를 구분한다.
구독자는 추가 JSON 필드를 무시할 수 있으며 다른 팀원 파일을 수정할 필요가 없다.

### 3. 연속 운전·반복 장애 자동 시험

```bash
source ~/vision-arm/env.sh
python scripts/check_team2_roundtrip.py --domain 97 \
  --duration 180 --restart-cycles 5 --interval 0.05 \
  --out logs/team2_stress_check.json
```

최소 100개 및 지정 시간 동안 정상 패킷을 보내고, 이후 서버 중단·재시작을 지정 횟수 반복한다.
매 장애에서 ACK 실패를 기록하고, 서버 복귀 후 새 패킷 전달·이전 실패 패킷 재전송 없음까지 확인한다.
각 패킷의 타깃·로그를 확인한 뒤 다음 패킷을 보내는 방식이므로 **응답 대기 없는 고속 카메라 입력 부하 시험이나 모든 프레임 무손실 보장은 아니다.**
시험은 전용 ROS 도메인에서 팀원 2 노드만 실행하며 물체 생성·로봇 이동은 요청하지 않는다.
장시간 확인이 더 필요하면 `--duration 1800` 등으로 늘릴 수 있다 (최대 3600초, 재시작 최대 20회).
3분 시험 통과를 수시간 운전의 안정성 보장으로 해석하지 않는다.

### 4. 검증 결과

- 전체 테스트: **103개, 오류·실패·skipped 0**.
- 합성 시계 시험: 전진/후진 점프, 프레임 사이 점프, 미래 capture, 샘플링 지연, 안정화 복구 확인.
- 전송 단계 시험: 검사 전 실패·송신 실패·정상 ACK를 구분하는 회귀 테스트 통과.
- 실제 보유 이미지 100회: ACK·동일 타깃 각 100/100, 통신 오류·시계 경고 0.
- 해당 파일 입력 시험의 프레임→ACK P95 **19.74ms**, TCP 왕복 P95 **0.64ms**. 이전 수치와 다른 실행 조건의 단일 시험 결과다.
- 산출물: `logs/team2_clock_stage_images_check.json`, `logs/team2_stress_check.json` 및 시험별 CSV.

남은 공동 검증은 실제 스마트폰 보정 데이터, GUI 표시, 로봇 타깃 소비 확인이다. 관련 담당 파일은 수정하지 않았다.

연속 시험 실제 결과: 정상 구간 **180.017초 / 3,371개**, ACK 및 동일 타깃 **3,371/3,371**, 통신 오류·시계 경고 0.
합성 패킷의 프레임→ACK P95 1.80ms, TCP 왕복 P95 0.59ms였다 (실제 영상 처리 성능이 아님).
서버 재시작 5회 모두 의도된 실패를 기록한 뒤 새 패킷으로 복구했고, 실패 패킷의 자동 재전송은 없었다.
보고서 recovery_ms는 새 서버 discovery 확인 후 복구 패킷 송신부터 타깃·로그 수신까지 약 62~73ms이며 전체 서버 재시작 소요 시간이 아니다.

# 3주차 팀원 3 (가영) — 스폰·흡착

2026-11-17, 가영 PC에서 구현·검증. 담당 파일 `ur5e_pick_place/object_spawner.py` 만 수정했고 테스트 `test/test_object_spawner.py` 를 새로 추가했다.
`pick_place_node.py`·`sim.yaml`·`setup.py`·URDF 등 다른 팀원 파일은 바꾸지 않았다.

## 구현

| 항목 | 내용 |
|---|---|
| 패킷 → 스폰 목록 (`plan_spawns`) | `calib_ok: false` 는 거부. `part_types` 에 없는 종류(unknown)·mm 가 null 인 물체는 건너뛰고 이유를 남김. 남은 물체를 순서대로 `obj_0`~`obj_{max_objects-1}` 에 배정. `grasp.work_to_base` 로 base_link(=world) 좌표, yaw = 각도 + 작업 평면 yaw (너트처럼 각도 null 이면 0°). 바닥과 겹치지 않게 1 mm 위에 놓음 |
| Gazebo 통신 (`GzSpawner`) | `ros2 run` 프로세스 대신 gz-transport 로 직접 `/world/workcell/create`·`/remove` 요청, `/vacuum/obj_k/state` 구독, `/vacuum/obj_k/detach` 발행 (ros_gz_bridge 불필요) |
| DetachableJoint 처리 | 처음 쓰는 슬롯은 스폰 즉시 자동으로 붙으므로 "attached" 를 기다렸다가 detach → "detached" 확인. 이미 한 번 뗀 슬롯은 기다리지 않음. 붙은 채 지우지 않도록 삭제 전에 떼어 둠 |
| 단독 실행 노드 | `python3 -m ur5e_pick_place.object_spawner`. `/robot/targets` 최신 패킷을 `/sim/spawn_objects` (Trigger) 호출 때 스폰(이전 obj_k 는 먼저 삭제), `/sim/clear_objects` 로 전부 삭제. 작업 평면·`max_objects`·`part_types` 는 `config/sim.yaml` 의 `pick_place_node` 값을 그대로 읽음 (`base_link→work_plane` TF 와 같은 값). `ros2 run` 용 entry point 는 `setup.py`(공유 파일)를 안 건드리려고 추가하지 않음 |

## DetachableJoint 동작 확인 (DECISIONS.md 4장 "지웠다 다시 만든 같은 이름 모델을 잡는지")

gz-sim8 `DetachableJoint.cc` 소스와 실기로 확인:
- 시뮬레이터 시작 때 붙이기 요청이 켜져 있어, 슬롯마다 **처음** 나타난 obj_k 만 자동으로 붙는다.
- 한 번 detach 한 뒤에는 같은 이름 모델을 지웠다 다시 만들어도 **자동으로 붙지 않는다**.
- 그 상태에서 attach 를 보내면 지워진 옛 모델 대신 **새 모델을 찾아 붙인다** (실기: 다시 만든 obj_0 이 팔과 함께 0.100 m 상승, detach 후 바닥으로).
- 붙어 있지 않을 때 detach 는 무시된다 (다음 attach 에 영향 없음).

## 실기 결과 (headless Gazebo + MoveIt, 보정된 형식의 합성 패킷)

| 시험 | 결과 |
|---|---|
| 새 슬롯 스폰: 나사(100, 50 mm, 30°)·unknown·너트(200, 150 mm)·나사(250, 40 mm, 135°) | 3개 스폰, unknown 건너뜀. Gazebo 위치 (0.400, −0.100)·(0.500, 0.000)·(0.550, −0.110) m, 각도 30°·0°·135° = 계산값과 일치. 0.7초 |
| 스폰 후 팔 이동 (`/sim/move_to_target`) | 부품 최대 이동 **0.0 m** → 패드에서 제대로 떨어져 있음 |
| 스폰 노드 재시작 후 같은 이름 재스폰 | 성공. 이전 상태를 모르는 슬롯마다 최대 2초 대기 → 4.6초 |
| 같은 노드로 재스폰 | 0.5~0.6초 |
| 다시 만든 obj_0 attach → 팔 0.10 m 상승 → detach | 0.100 m 따라 올라갔다가 바닥으로 떨어짐 |
| `calib_ok: false` 패킷 | 거부 메시지, 스폰 안 함 |
| 삭제 | obj_0~obj_2 모두 사라짐 |
| colcon test | **112 tests, 0 failures** (`test_object_spawner.py` 6개 추가: 좌표·각도 변환, 회전된 평면, 미보정 거부, 건너뛰기·슬롯 배정, 빈 패킷, sim.yaml·모델·URDF 슬롯 수 일치) |

실제 폰·보정 패킷으로는 아직 안 해 봤다 (동경 보정 대기). 스폰은 부품을 인식 위치에 놓는 것까지이며 집기 사이클은 아니다.

### 가까이 집어 옮기기 시연 (11-17, Gazebo 창 켜고 2회 반복)

위 시험은 패드를 부품 0.15 m 위에 둔 채 attach 해서, 부품이 패드에 닿지 않고 그 거리만큼 떨어진 채 따라왔다 (DetachableJoint 는 attach 순간의 상대 위치를 그대로 고정한다).
실제 흡착처럼 보이도록 스폰한 부품 바로 위까지 내려가 집어 옮겼다. 팔은 `/sim/move_to_target` 의 목표 파라미터를 바꿔 가며 단계별로 움직였다 (확인용 임시 스크립트, 저장소에 넣지 않음).

순서: 부품 위 80 mm 접근 → 패드 끝을 부품 바로 위까지 하강 → attach → 100 mm 로 들어 올림 → 옆으로 이동 → 하강 → detach → 위로 빠짐. 나사는 장축 각도(30°)에 패드를 맞췄다.

| 부품 | 하강 높이 (패드 끝) | 들어 올렸을 때 부품 높이 | 내려놓은 위치 (m) | 목표 (m) |
|---|---|---|---|---|
| 나사 30° (100, 105 mm) | 12 mm | 89 mm / 88 mm (패드 끝 100 mm 바로 아래) | (0.461, −0.090) / (0.460, −0.091) | (0.460, −0.090) |
| 너트 (200, 150 mm) | 10 mm | 90 mm / 91 mm | (0.450, 0.010) / (0.449, 0.010) | (0.450, 0.010) |

모든 이동이 첫 시도에 계획·실행 성공. MoveIt 은 스폰한 부품을 충돌 물체로 모르므로 하강 높이는 부품 높이(나사 8 mm, 너트 6 mm)보다 조금 높게 잡았다.
자동 집기 사이클(`/sim/run_cycle`)이 아니라 이동·흡착·해제가 이어서 동작하는지 본 확인이다.

- Gazebo 창을 닫으면 Gazebo 서버도 함께 꺼지지만 MoveIt·브리지 노드는 남는다. 이 상태에서 스폰은 "삭제 요청 실패", 이동은 `CONTROL_FAILED` 로 실패하므로 launch 전체를 다시 띄운다.
- 부품이 작아 기본 시점에서는 잘 안 보인다. 작업 시트 쪽을 마우스 휠로 확대하거나 Entity Tree 에서 `obj_0` → Move to 로 본다 (`/gui/move_to/pose` 로 카메라를 옮기는 요청은 실패해서 쓰지 않았다).

## 실행

```bash
ros2 launch ur5e_pick_place sim.launch.py gazebo_gui:=false launch_rviz:=false   # 터미널 1
python3 -m ur5e_pick_place.object_spawner                                         # 터미널 2 (env.sh, install/setup.bash source 후)
ros2 service call /sim/spawn_objects std_srvs/srv/Trigger                         # 터미널 3: 최신 /robot/targets 패킷대로 스폰
ros2 service call /sim/clear_objects std_srvs/srv/Trigger
```

## 남은 일 (팀원 3)

- 실제 보정 패킷으로 "카메라 앞 부품 위치대로 Gazebo 에 부품이 생김" 확인 (동경 보정 후, 팀원 2 `tcp.launch.py` 의 `/robot/targets` 사용).
- TF: 스폰한 obj_k 를 RViz 에서 보이게 할지 결정 (`work_plane` static TF 는 이미 sim.launch.py 에 있음).
- `pick_place_node` 의 `/sim/run_cycle` 2단계에서 `plan_spawns`·`GzSpawner.spawn_all` 을 부르도록 연결 — `pick_place_node.py` 담당자와 협의 필요.

## 팀원 2 ↔ 최신 팀원 3 스폰 코드 연결 (2026-11-17)

다른 팀원의 object_spawner.py, camera_stream.py, 보정·GUI·로봇 제어 원본은 수정하지 않았다. 팀원 2용 연결 런치·세션 확인 모듈·검사 스크립트만 추가했다.

### 연결 보완의 이유

현재 sim.launch.py는 실행마다 새 GZ_PARTITION을 쓴다. 별도 터미널에서 팀원 3 스폰 노드를 그대로 실행하면 Gazebo가 있는 partition을 모를 수 있다. ROS /robot/targets는 받아도 Gazebo 삭제·생성 서비스에 연결되지 않는 원인이 된다.
`team2_link.launch.py`는 최신 logs/workcell_session.json에서 실행 중 월드·partition을 읽고, 동일한 환경으로 기존 팀원 3 스폰 모듈을 실행한다. 기존 tcp.launch.py도 함께 실행한다.
ROS 도메인이 다르거나 Gazebo가 이미 종료됐으면 시작 단계에서 설명과 함께 실패한다. 다른 팀원 코드를 복사·재작성하지 않았다.

### 실제 입력으로 사용하는 순서

터미널 1 (기존 동일 시뮬레이션은 Ctrl+C로 종료):

```bash
source ~/vision-arm/env.sh
unset GZ_PARTITION
ros2 launch ur5e_pick_place sim.launch.py
```

터미널 2:

```bash
source ~/vision-arm/env.sh
ros2 launch vision_pick_place team2_link.launch.py
```

위 명령은 TCP 서버·브리지와 수정하지 않은 기존 object_spawner를 함께 실행한다. tcp.launch.py, integration.launch.py, 기존 단독 스폰 노드를 추가로 중복 실행하지 않는다. GUI와 카메라·자동 집기는 시작하지 않는다.

터미널 3 (영상·보정 담당이 준비한 실제 설정):

```bash
source ~/vision-arm/env.sh
ros2 run vision_pick_place vision_node --ros-args \
  --params-file src/vision_pick_place/config/params.yaml
```

터미널 4:

```bash
source ~/vision-arm/env.sh
ros2 topic echo /robot/targets --once
ros2 service call /sim/spawn_objects std_srvs/srv/Trigger "{}"
# 필요할 때 삭제
ros2 service call /sim/clear_objects std_srvs/srv/Trigger "{}"
```

스폰하려면 실제 내부 보정·ArUco 작업 평면 보정과 영상 입력이 준비되어 calib_ok=true여야 한다. 기본 MVTec 입력은 미보정이므로 스폰 거부가 정상이다. 통과시키려고 임의로 calib_ok만 true로 바꾸지 않는다.

### 실제 Gazebo 연동 검증 결과

전용 ROS_DOMAIN_ID=108에서 새로운 Gazebo와 team2_link.launch.py를 띄웠다. 기존 사용자의 실행은 종료하지 않았다.

- SDF/Gazebo/MoveIt 상자 위치: screw (0,+0.85,0)m, nut (0,-0.85,0)m 모두 일치.
- 합성 미보정 패킷: ACK 후 스폰 거부, mm 없는 좌표로 물체 생성 안 함.
- 합성 보정 패킷: TCP ACK 성공, /robot/targets가 입력과 동일.
- 나사 (100,105)mm, 30° → obj_0 실제 XY 약 (0.400,-0.045)m.
- 너트 (200,150)mm, angle=null → obj_1 실제 XY 약 (0.500,0.000)m.
- unknown 객체 1개는 건너뜀.
- 두 번의 스폰/재스폰 후 최신 Gazebo pose stream에서 실제 위치 확인. 삭제 서비스 후 scene/info에 모델이 없는 것까지 확인.
- 실제 폰 보정값·방향 정확도·흡착 이동·±0.85m 상자 투입·전체 집기 사이클은 이번 검증에 포함하지 않았다. 팔 이동 명령은 보내지 않았다.
- 증거: logs/team2_spawn_link_check.json, logs/workcell_layout_check.json (git 제외).

재현 검사: 위 실행이 준비된 상태에서 `python scripts/check_team2_spawn_link.py`. 실제로 받은 최신 보정 패킷의 물체를 생성·재생성·삭제하므로 빈 시험 장면에서 사용한다.
합성 시험은 전용 nonzero ROS_DOMAIN_ID에서만 `--demo`를 붙여 사용한다. 다른 영상 발행자가 있으면 demo를 거부한다. 합성 시험 통과는 실제 카메라 보정 정확도 검증이 아니다.

### 오류 설명과 남은 테스트 불일치

| 오류/증상 | 원인·대응 |
|---|---|
| 세션 파일 없음 | 새 sim.launch.py부터 실행. 경로는 프로젝트 루트 기준 |
| ROS_DOMAIN_ID mismatch | 시뮬레이터와 연결 터미널의 ROS 도메인을 같게 설정 |
| Recorded Gazebo session is not responding | Gazebo 종료 또는 오래된 세션 기록. launch 전체 재시작 |
| /sim/spawn_objects 없음 | 연결 런치의 object_spawner 시작 로그 확인. 기존 노드 중복 실행 금지 |
| calib_ok false | 실제 보정·영상 준비 필요. TCP 오류가 아니라 올바른 스폰 거부 |
| 삭제/스폰 요청 실패 | Gazebo 서버·partition·중복 obj_k 확인 |
| 테스트 expected Y=0.45, actual Y=0.85 | 상자 배치 변경에 비해 기존 test_workcell_scene.py 기대 좌표가 오래됨 |

현재 전체 테스트 결과는 **113개 중 112개 통과, 1개 실패**다.
실패는 `test_workcell_has_floor_and_both_bins`에서 옛 ±0.45m를 하드코딩했기 때문이며 실제 장면은 ±0.85m다.
이 테스트는 타 담당 파일이므로 요청 범위를 지키기 위해 수정·skip하지 않았다. 해당 담당자가 기대값을 현재 설정 또는 고정 fixture 기준으로 갱신해야 한다. 전체 테스트가 통과했다고 기록하지 않는다.

새 코드와 이번 기록은 로컬 변경 상태다. 커밋·푸시는 아직 진행하지 않았다.
