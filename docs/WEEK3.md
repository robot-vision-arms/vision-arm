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
| Gazebo 통신 (`GzSpawner`) | gz-transport 로 `/vacuum/obj_k/state`·`/world/workcell/pose/info` 구독, `/vacuum/obj_k/detach` 발행 (ros_gz_bridge 불필요). `/world/workcell/create`·`/remove` 요청만 `gz service` 명령으로 보냄 (아래 '알게 된 것') |
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
실제 흡착처럼 보이도록 스폰한 부품 바로 위까지 내려가 집어 옮겼다. 팔은 `/sim/move_to_target` 의 목표 파라미터를 바꿔 가며 단계별로 움직였다. 같은 확인을 `scripts/check_spawn_week3.py` 로 정리했다 (아래 '확인 스크립트').

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

### 확인 스크립트

시뮬레이터를 띄운 뒤 `python scripts/check_spawn_week3.py` (스폰 노드가 없으면 스크립트가 직접 띄웠다 끈다).
스폰 위치(+TF·마커) → 미보정 거부 → 팔이 지나가도 부품 그대로 → 나사·너트 바로 위에서 집어 옮기기(+TF 높이) → 같은 이름 재스폰 → 삭제 시 마커 제거 를 차례로 확인하고, 하나라도 어긋나면 그 자리에서 멈춘다. 결과는 `logs/week3_spawn_check.json`. 팔을 움직이고 부품을 남긴다.

11-17 headless 실행: **통과**. 팔 통과 시 부품 이동 0.0 mm, 나사 들어 올림 88 mm·놓인 위치 오차 0.6 mm, 너트 90 mm·0.8 mm, 재스폰 성공.

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
- ~~TF~~ 11-17 완료 (아래).

## TF·RViz 표시 (11-17)

스폰 노드가 Gazebo 의 실제 부품 위치를 ROS 로 내보낸다. 수정 파일은 `object_spawner.py`, `test_object_spawner.py`, `scripts/check_spawn_week3.py`.

| 항목 | 내용 |
|---|---|
| TF `base_link → obj_k` | `/world/workcell/pose/info` 의 obj_k 위치·자세를 10 Hz (`tf_rate_hz`) 로 발행. 흡착해서 들면 따라 움직이고, 부품을 지우면 0.5초 안에 그 슬롯 발행을 멈춤. `sim.launch.py` 의 `base_link → work_plane` 과 합쳐 `work_plane → obj_k` 를 바로 읽을 수 있음 |
| 마커 `/sim/object_markers` | `visualization_msgs/MarkerArray`. obj_k 프레임에 붙인 원기둥(나사 지름 8×길이 40 mm 를 장축 방향으로 눕힘, 너트 지름 13×두께 6 mm). 모양·색은 `models/<type>/model.sdf` 의 visual 에서 읽음 (종류 추가 시 코드 수정 없음). 지우면 DELETE |
| 시계 | `sim.yaml` 의 `use_sim_time` 을 따라 /clock 기준 시각으로 발행 (RViz·MoveIt 과 같은 시계) |
| RViz 에서 보기 | Add → TF (obj_0~2 표시), Add → MarkerArray → Topic `/sim/object_markers` |
| 테스트 | 3개 추가: RPY→쿼터니언, 나사·너트 model.sdf 에서 모양 읽기, box·모르는 형상(mesh) 처리 → 전체 **115 tests, 0 failures** |

확인 스크립트에 TF 단계를 넣고 headless 로 실행 → **통과**:
- 스폰 직후 TF `work_plane → obj_k` = (100.0, 105.0) mm 30.0°, (200.0, 150.0) mm 0.0°, (230.0, 50.0) mm 135.0° — 패킷 mm·각도와 일치
- 들어 올렸을 때 TF 높이 = Gazebo 높이 (나사 88.8 mm, 너트 89.9 mm)
- 지우면 마커 3개 모두 DELETE

### 알게 된 것

- **gz-transport Python 요청이 들쭉날쭉 시간 초과:** 위치 구독을 넣은 뒤 스폰 노드의 삭제 요청이 응답을 못 받는 일이 생겼다 (없는 이름 `foo` 도 실패, 같은 프로세스의 다른 노드도 실패/성공이 섞임). 같은 요청을 `gz service` 명령으로는 매번 0.3초에 성공. 원인은 못 좁혔고, 요청만 `gz service` 별도 프로세스로 보내도록 바꿔 해결 (스폰 3개 + 삭제 3개에 약 2초). 실패하면 한 번 더 보냄.
- **시뮬레이터 직후 팔 이동 실패:** 확인 중 한 번, `/sim/move_to_target` 의 계획 재시도 3회가 모두 `99999` 로 실패 (동경 기록의 OMPL 초기 실패와 같은 현상). 확인 스크립트는 이동을 한 번 더 부르도록 함. `pick_place_node.py` 는 수정하지 않음.
- 확인 스크립트가 통과해도 종료 시 gz·tf 백그라운드 스레드 때문에 core dump(종료 코드 134)가 나서, 결과 저장 뒤 `os._exit` 로 끝낸다.

## 진공 컵 그리퍼와 현실 흡착 조건 (11-17)

DetachableJoint 는 attach 명령만 받으면 거리와 상관없이 그 순간의 상대 위치로 붙인다. 그래서 패드가 부품 150 mm 위에 있어도 부품이 따라 올라왔다 (현실의 그리퍼에서는 불가능).
진공 컵과 자석을 비교한 뒤, 여러 재질에 쓸 수 있고 산업 현장에서 가장 많이 쓰는 **진공 컵**으로 정했다 (자석은 쇠 나사·너트에는 유리하지만 비자성 부품으로 확장할 수 없고 여러 개가 딸려 옴).

### 그리퍼 모양 (`urdf/ur5e_vacuum.urdf.xacro`, 팀원 2 파일 — 사용자 승인으로 수정)

40 × 12 mm 직육면체 패드 → tool0 에서 아래로 **홀더(지름 16 mm, 36 mm) → 공기 관(지름 3 mm) → 주름형 진공 컵(지름 5 mm, 높이 6 mm)**.
전체 길이 50 mm·링크 이름 `vacuum_pad` 는 그대로라 `sim.yaml` 의 `tool_length_m`·SRDF 충돌 설정은 바꾸지 않았다.

### 진공 ON/OFF 서비스 `/sim/vacuum` (std_srvs/SetBool)

현실 그리퍼처럼 "obj_0 을 붙여라" 가 아니라 "진공을 켜라" 만 명령한다. 켜는 순간 컵 아래 부품이 아래 조건을 모두 만족하면 그 부품 하나만 attach, 아니면 아무것도 붙이지 않고 가장 가까운 부품의 실패 이유를 돌려준다. `data: false` 는 붙어 있던 부품을 떼어 중력으로 떨어뜨린다.

| 조건 | 기본값 (파라미터 `vacuum_*`) | 근거 |
|---|---|---|
| 컵이 아래를 봄 | 연직에서 10° 이내 | 기울면 컵 입술이 면에 밀착 안 됨 |
| 컵 전체가 부품 면 위 | 끝·가장자리·구멍 밖으로 나가지 않음 | 입술 일부가 허공이면 공기가 샘 |
| 둥근 면(나사) 밀착 | 컵 입술이 닿는 면의 높이 차 ≤ 2 mm (`lip_compliance`) | 주름 컵이 줄어들어 메울 수 있는 정도. 지름 8 mm 나사에서 옆으로 약 ±0.9 mm 까지 |
| 컵 끝 ~ 부품 면 간격 | −3 ~ +2 mm (`max_press`, `seal_gap`) | 조금 떨어져도 빨아들이고, 더 누르면 충돌 |
| 흡착력 ≥ 2 × 무게 | 60 kPa × 컵 면적 = 1.18 N, 나사 0.20 N (`vacuum_kpa`, `safety_factor`) | 들어 올릴 때 가속·흔들림 여유 |

- 둥근 컵이라 툴 방향(yaw)은 맞출 필요가 없다 (이전 직사각형 패드의 장축 정렬 조건은 없앰).
- **너트는 진공 컵으로 못 잡는다:** M8 너트(맞변 13 mm)는 가운데 구멍(지름 8 mm)이 있어 구멍 둘레 폭이 2.5 mm 뿐이다. 지름 5 mm 컵은 어디에 놓아도 구멍이나 바깥으로 걸쳐 공기가 샌다. Gazebo 너트 모델에는 구멍이 없어 `HOLE_RADIUS_M` 으로 따로 반영했다. → 아래 **컵 두 개 헤드**로 해결 (작은 컵을 테두리에 올림).
- 공기 흐름·누설·흡착력의 동적 변화는 계산하지 않는다 (Gazebo 에 없음). 위 조건은 그 대신 쓰는 기하·힘 검사다.
- 기존 `/vacuum/obj_k/attach·detach` 토픽(sim.launch.py 브리지, `pick_place_node.py` 가 사용)은 남아 있어 조건 없이 붙일 수 있다. `/sim/run_cycle` 이 `/sim/vacuum` 을 쓰도록 바꾸는 건 `pick_place_node.py` 담당과 협의 필요.

```bash
ros2 service call /sim/vacuum std_srvs/srv/SetBool "{data: true}"    # 진공 ON
ros2 service call /sim/vacuum std_srvs/srv/SetBool "{data: false}"   # 진공 OFF
```

### 확인 (headless, `check_spawn_week3.py`) → **통과**

| 상황 | 결과 |
|---|---|
| 부품 150 mm 위에서 진공 ON | 거부: "부품과 떨어져 있음 (간격 142.7 mm > 2 mm)", 부품 그대로 |
| 너트 가운데(구멍 위)·테두리 위, 컵 끝 7 mm | 둘 다 거부: "구멍 둘레 폭 2.5 mm 가 컵 지름 5 mm 보다 좁아 밀봉 불가", 너트 그대로 |
| 나사 30°, 툴 30°, 컵 끝 9 mm | 흡착: 간격 1.5 mm, 중심에서 0.1 mm, 흡착력 1.18 N / 무게 0.20 N → 91 mm 들어 올려 옮김, 놓인 위치 오차 0.7 mm |
| 나사 135°, **툴 0°** (방향 안 맞춤) | 흡착: 간격 1.8 mm, 중심에서 0.5 mm → 옮김, 오차 0.9 mm |
| 재스폰·삭제 시 마커 제거 | 통과 |

**Gazebo·RViz 창을 띄우고 다시 실행 (11-17):** 창을 띄우면 시뮬레이터가 느려져 팔이 멈추는 높이가 목표보다 1~2 mm 높아졌다 (컵 끝 9 mm 목표 → 간격 2.6 mm 로 거부).
실제 진공 그리퍼처럼 **진공이 안 잡히면 1 mm 씩 더 내려가며 다시 켜도록** 확인 스크립트를 바꿨다 (목표 간격 0.5 mm, 최대 2 번). 결과 **통과**:
나사 30° 는 첫 시도에 흡착(간격 1.5 mm, 놓인 위치 오차 1.3 mm), 나사 135° 는 첫 시도 간격 3.2 mm 로 거부 → 1 mm 내려가 흡착(간격 0.5 mm, 오차 0.8 mm). 너트 2회 거부, 150 mm 거부는 같음.
스폰 노드를 Ctrl+C 로 끌 때 받던 메시지 변환 오류(traceback)가 나던 것도 종료 중일 때는 무시하도록 고쳤다.

URDF 변경 영향 확인: 팀원 1 의 `scripts/check_sim_week2.py` 를 새 시뮬레이터에서 실행 → 1회차는 첫 이동에서 손목 관절 추종 오차(0.208 rad > 0.2)로 `CONTROL_FAILED`, 바로 다시 실행한 2회차는 **통과** (지정 좌표 이동, 늦게 스폰한 부품 attach→detach→attach 0.101 m 상승→detach). 1회차 실패가 컵 모양 때문인지 시뮬레이터 시작 직후 일시 현상인지는 더 지켜볼 것.

테스트: 진공 컵 판정 10개 (방향 무관 흡착, 150 mm·4 mm 떨어짐·누름·끝 걸침·둥근 면 밀착 실패·면 벗어남 거부, 무게·기울기, 너트 구멍·작은 컵) → 전체 **125 tests, 0 failures**.

### 컵 두 개 헤드(5 mm 컵 + 링 컵)로 너트까지 흡착 (11-17)

컵 하나로는 나사와 너트를 둘 다 잡을 수 없다. 둘 다 현실 무게 그대로 계산했다 (모델 파일: 나사 M8×40 20 g, 너트 M8 5 g. 표준 규격 무게와 비슷함).
너트는 구멍 지름이 8 mm 라 5 mm 컵을 가운데에 대면 입술이 전부 구멍 위에 떠서 밀봉 자체가 안 되고, 테두리 폭은 2.5 mm 라 5 mm 컵이 올라가지 못한다.
나사를 들려면 컵 지름이 2.9 mm 이상이어야 해서, 테두리용 작은 컵 하나로 둘 다 잡을 수도 없다.

**1차 시도: 2 mm 마이크로 컵을 테두리에 (버림).** 테두리(2.5 mm)에 2 mm 컵이라 위치 여유가 ±0.25 mm 뿐인데 팔이 멈추는 위치는 ±0.4~0.8 mm 흔들린다. 보정 없이 내리면 구멍 위로 0.75 mm 걸쳐 거부됐고, 하강 뒤 컵 위치를 재서 3 번 보정해도 오차가 줄지 않았다. 마지막에 허용 범위 끝에서 한 번 잡혔지만 될 때도 있고 안 될 때도 있는 수준이라 버렸다.

**2차: 센터링 핀 달린 링 컵 (채택).** 와셔·너트처럼 구멍 있는 부품에 실제로 쓰는 방식이다.

| | 5 mm 컵 (나사용) | 링 컵 (너트용) |
|---|---|---|
| 위치 | tool0 바로 아래 | tool0 +X 12 mm, 입술 끝 높이 같음 |
| 모양 | 주름형 컵 지름 5 mm | 입술 두 겹 (안 지름 9 mm, 바깥 12 mm), 가운데는 막힘. 끝이 뾰족한 센터링 핀 지름 7.6 mm (스프링) |
| 진공 면적 | 지름 5 mm 원 | 두 입술 사이 고리 (반지름 4.5 ~ 6 mm) |
| 흡착력 (60 kPa) | 1.18 N > 나사 2 × 0.196 N | 2.97 N > 너트 2 × 0.049 N |
| 위치 여유 | 나사 윗면 옆으로 ±0.9 mm | 핀이 ±1.5 mm 어긋남까지 구멍에 끌려 들어가 너트를 가운데로 맞춤 (남는 틈 0.2 mm) |

- 핀은 구멍보다 0.4 mm 가늘어 핀만으로는 공기가 새므로, 밀봉은 컵 가운데를 막고 입술 두 겹이 테두리 위에서 한다. 핀은 위치를 맞추는 역할만 한다.
- 실제 다중 컵 헤드처럼 컵마다 밸브가 따로 있다고 보고, `/sim/vacuum` 은 (컵, 부품) 쌍마다 검사해 통과한 컵 하나만 켠다. 응답에 어느 컵으로 잡았는지 나온다.
- URDF: 홀더 옆 팔 → 관 → 링 컵. 핀은 모양만 보이고 충돌은 없다 (Gazebo 너트 모델에 구멍이 없어 핀에 충돌을 주면 너트 윗면을 누름). xacro 주석에 `: ` 를 쓰면 robot_description 이 YAML 로 읽혀 launch 가 실패하므로 주석에서 뺐다.
- `object_spawner.py`: `cup_tips()` 가 두 컵 끝 위치를 계산, `suction_check(..., ring=True)` 가 링 컵 판정 (핀이 구멍에 들어가는지, 맞춘 뒤 안쪽 입술이 구멍에 걸리지 않는지, 바깥 입술이 너트 밖으로 나가지 않는지, 고리 면적 흡착력). 파라미터 `vacuum_ring_outer_diameter`(0.012), `vacuum_ring_inner_diameter`(0.009), `vacuum_pin_diameter`(0.0076), `vacuum_pin_capture`(0.0015), `vacuum_ring_offset`(0.012).
- **시뮬레이션 한계:** 핀이 너트를 가운데로 밀어 맞추는 동작은 Gazebo 에서 일어나지 않는다 (판정으로만 반영). 그래서 잡힌 너트에는 팔의 어긋남(0.4~0.9 mm)이 그대로 남아 놓인 위치 오차에 섞인다.
- `/sim/run_cycle`(`pick_place_node.py`) 에서 너트를 링 컵으로 집으려면 툴 목표를 너트 중심에서 툴 −X 로 12 mm 옮겨야 하므로 담당자와 협의 필요. 확인 스크립트는 직접 계산해서 보낸다.

**확인 (Gazebo·RViz 창 띄움, `check_spawn_week3.py` 5 회 반복) → 5 / 5 통과**

| 회차 | 너트 (링 컵) 중심에서 어긋남 → 놓인 위치 오차 | 나사 30° 오차 | 나사 135° 오차 |
|---|---|---|---|
| 1 | 0.8 mm → 1.0 mm | 0.4 mm | 0.2 mm |
| 2 | 0.7 mm → 1.2 mm | 0.3 mm (1 mm 더 내려가서) | 1.3 mm |
| 3 | 0.9 mm → 1.5 mm | 0.2 mm | 0.5 mm |
| 4 | 0.5 mm → 0.5 mm | 0.5 mm (1 mm 더 내려가서) | 0.8 mm (1 mm 더 내려가서) |
| 5 | 0.4 mm → 0.6 mm (1 mm 더 내려가서) | 0.6 mm | 0.8 mm |

- 너트 5 / 5 흡착. 위치 보정 없이 바로 잡혔다 (팔 어긋남 최대 0.9 mm < 핀 허용 1.5 mm). 5 회차의 1 mm 추가 하강은 높이(간격 2.3 mm) 때문이다.
- 나사 10 / 10 흡착. 놓인 위치 오차는 모두 허용치 5 mm 안.
- 매 회 150 mm 위 진공 ON 거부, 5 mm 컵을 너트 가운데에 대면 "구멍 둘레 폭 2.5 mm 가 컵 지름 5 mm 보다 좁아 밀봉 불가" 로 거부 (의도한 결과), 재스폰·삭제 통과.

테스트: 두 컵 위치(툴을 돌리면 링 컵도 같이 돎), 링 컵 너트 흡착(핀 허용 1.4 mm 까지, 1.6 mm 거부, 3 mm 위 거부, 흡착력 2.97 N), 링 컵 나사 거부·굵은 핀·좁은 안쪽 입술 거부 → 전체 **128 tests, 0 failures**.

## 팀 코드 연동 확인 (11-17, git pull 후)

`git pull` 로 팀원 2 의 세션 연결(`team2_link.launch.py`, 실행마다 새 `GZ_PARTITION` 을 `logs/workcell_session.json` 에 기록), 상자 위치 ±0.85 m 변경을 받은 뒤, 내 진공 컵 변경과 함께 전용 `ROS_DOMAIN_ID=37`·창 없는 새 시뮬레이터에서 확인했다. 다른 팀원 파일은 수정하지 않았다.

- `check_spawn_week3.py` 변경 (내 파일): `GZ_PARTITION` 이 없으면 `logs/workcell_session.json` 에서 읽는다. 새 sim.launch.py 는 실행마다 partition 이 바뀌어서, 이게 없으면 스크립트의 Gazebo 구독이 다른 partition 을 본다.

| 확인 | 결과 |
|---|---|
| 전체 빌드 | 2 개 패키지 성공 |
| 전체 테스트 | 129 개 중 128 통과, 1 실패: `test_workcell_scene.py::test_workcell_has_floor_and_both_bins` 가 옛 상자 Y ±0.45 m 를 기대 (실제 ±0.85 m). 팀원 2 가 위에 기록한 것과 같은 원인, 타 담당 파일이라 수정하지 않음 |
| `check_workcell_layout.py` (팀원 2) | ok: SDF·Gazebo·MoveIt 상자 위치 일치 |
| `team2_link.launch.py` + `check_team2_spawn_link.py --demo` (팀원 2) | ok: TCP ACK → `/robot/targets` 동일 → 내 스폰 노드가 나사·너트 생성, 미보정 패킷 거부, 삭제 성공. 링 컵 변경 후 스폰 노드에 `/sim/vacuum` 도 같이 뜸 |
| `check_spawn_week3.py` (팀원 3, 같은 스택에서) | **통과**: 너트 링 컵 흡착(중심에서 0.8 mm) → 오차 1.1 mm, 나사 30°·135° 5 mm 컵 → 0.8·0.3 mm, 거부 2 건 정상 |
| `check_sim_week2.py` (팀원 1) | 새 시뮬레이터에서 **통과** (지정 좌표 이동, 늦게 스폰한 부품 attach → 0.100 m 상승 → detach). 컵 모양 변경 영향 없음 |
| `check_team2_repeat_motion.py --execute --cycles 3` (팀원 2) | ok: 3 / 3 반복 이동, 파라미터 복구 |

연동 중 알게 된 점 (담당자에게 공유할 것):
- `check_sim_week2.py` 는 `logs/workcell_session.json` 을 읽지 않아, 새 sim.launch.py 에서는 `GZ_PARTITION` 을 직접 export 해야 Gazebo 상태를 받는다 (안 하면 대기 시간 초과).
- 같은 스크립트는 obj_0 이 한 번도 없던 새 시뮬레이터에서만 통과한다 (DetachableJoint 는 처음 생긴 obj_k 만 자동 attach). 다른 확인으로 obj_0 을 만든 뒤 실행하면 "상태 대기 시간 초과" 가 난다. 스크립트 docstring 에 적힌 전제대로이며 오류가 아니다.
- ±0.85 m 상자에 실제로 내려놓는 동작(`/sim/run_cycle`)은 이번에 확인하지 않았다.

## 실제 폰 영상 → 스폰 노드 연결 확인 (11-17, 보정 전)

보정 파일이 들어오면 바로 완료 기준("카메라 앞 부품 위치대로 Gazebo 에 부품이 생김")을 확인할 수 있도록, 실제 폰 영상으로 끝까지 이어지는지 먼저 확인했다. 캘리브레이션은 팀원 1(동경)이 진행한다.

실행: `sim.launch.py`(창 없이) → `team2_link.launch.py`(robot_server·robot_bridge·object_spawner) → `vision_node`(`params.yaml`, `camera_url:=http://172.20.10.1:8080/stream.mjpg`). ROS_DOMAIN_ID 0.

| 단계 | 결과 |
|---|---|
| 폰 영상 → `vision_node` | 수신. "미보정: px만 발행" 경고 (intrinsics·homography 없음) |
| `/vision/detections` | 약 7.5 Hz, `calib_ok: false`, `x_mm`·`y_mm`·`angle_deg` null, `u_px`·`v_px` 있음 |
| robot_bridge → TCP → robot_server | `/robot/log`: sent·acked·validated true, 지연 약 5.5 ms |
| `/robot/targets` | 약 7.5 Hz, 검출 패킷과 같은 내용 |
| `/sim/spawn_objects` | 거부: "calib_ok 가 false 인 패킷은 스폰하지 않습니다 (mm 좌표 없음)" (정상) |

- 이번 장면에는 시트 위에 부품이 없었는데도 12 개(나사 6·너트 6)가 검출됐다. 마우스·케이블·ArUco 마커·책상 물건이 잡힌 오검출이다. 작업 시트 안쪽만 보는 `sheet_roi` 는 보정이 있어야 켜지므로 보정 후 다시 확인한다.
- 스폰 노드는 패킷당 앞에서부터 최대 3 개(`max_objects`)만 스폰하므로, 오검출이 남아 있으면 진짜 부품이 밀려날 수 있다.
- 폰 송출 앱이 화면 꺼짐·백그라운드 전환 때 여러 번 멈췄다 (포트는 열려 있는데 프레임 0, 또는 포트 닫힘). 앱을 완전히 종료 후 다시 켜면 복구. 촬영·시연 중에는 자동 잠금을 끈다.

보정 후 할 일: `data/calib/intrinsics.npz`·`homography.npz` 가 생기면 같은 실행으로 부품 2~3 개를 시트 격자의 알려진 위치에 놓고 `/sim/spawn_objects` → Gazebo 위치와 놓은 위치(mm) 비교.
