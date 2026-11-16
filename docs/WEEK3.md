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
