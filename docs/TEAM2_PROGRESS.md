# 팀원 2 (성민) — 1·2주차 실행 결과

확인일: 2026-11-13. 실행 위치: `~/vision-arm`.
최신 공유 코드의 구현을 이 PC에서 재현했다. 화면 직접 확인과 자동 검증을 구분한다.

## 1주차: 프로젝트 작업 셀

| 항목 | 이번 확인 결과 |
|---|---|
| 빌드 | 두 패키지 `colcon build --symlink-install` 성공 |
| 로봇·매트·상자 | 실행 중 Gazebo scene/info에서 ur, work_mat, screw_bin, nut_bin 확인 |
| 흡착 패드 | 로봇 visual에 vacuum_pad 포함 확인 (고정 관절로 wrist_3_link에 합쳐짐) |
| 매트 중심 | (0.4485, -0.045, 0.0005)m, 설정 원점과 A4 크기로 계산한 중심과 일치 |
| 나사 상자 중심 | (0, 0.45, 0)m, sim.yaml과 일치 |
| 너트 상자 중심 | (0, -0.45, 0)m, sim.yaml과 일치 |
| 프로젝트 계획·실행 | /sim/move_to_target 서비스로 성공. tool0 목표 (0.4, -0.05, 0.30)m → (0.4, -0.05, 0.40)m |
| 기본 예제 RViz | 이전 사용자 확인에서 Plan·Execute 및 Gazebo 팔 이동 성공 |
| 프로젝트 GUI | 이번 시험은 headless. 프로젝트 화면 및 RViz 수동 조작은 사용자 확인 필요 |

처음 지정 좌표 이동은 5초 OMPL 경로 탐색에 실패했다. 같은 조건 재시도는 성공했다.
당시에는 이동의 반복 성공률·충돌 안전성까지 입증한 것은 아니었다. 이후 지정한 두 자세의 5회 왕복 검증은 아래 2026-11-17 기록에서 통과했다. 임의의 RViz 목표나 전체 작업 영역의 안정성을 보장하지 않으며 STL 충돌 메시·센서 미설정 Octomap 오류는 추가 검증 대상이다.

기존 로컬 suppress_child_warning 설정과 최신 공유 설정이 중복되어 로컬 중복을 제거했다.
공유 코드의 경고 억제·상태 토픽·로봇 이름 수정은 유지했다.

## 2주차: 검출·분류 로직 검증

35 tests, 0 errors, 0 failures, 0 skipped. 최초 샌드박스 시험은 ROS 기본 로그 경로의 쓰기 제한으로 실패했지만 권한 제한 밖에서 동일 테스트를 재실행해 통과했다.
회전 물체 중심·장축, 작은 잡음 제거, 너트 구멍, 빈 영상, 퇴화 윤곽선, 길이 비율 분류 경계 등의 기존 테스트를 포함한다.

### validation에서 설정 선택

Blur 5, 최소 면적 500px²를 고정하고 validation split 55장에서 비교했다.

| threshold | F1 |
|---|---:|
| 60 | 0.2588 |
| 80 | 0.4396 |
| 100 | **0.5083** |
| 120 | 0.3757 |
| Otsu | 0.0923 |

기존 설정 100의 선택을 재현했다. test split을 설정 선택에 사용하지 않았다.

### 별도 test split 결과

| 항목 | 결과 |
|---|---:|
| 이미지 / 정답 물체 | 60장 / 661개 |
| 예측 / IoU≥0.5 대응 | 979개 / 419개 |
| 정밀도 | 42.80% |
| 재현율 | 63.39% |
| 중심 MAE | 14.62px |
| 장축 각도 MAE | 5.46° (314개) |

오차는 대응된 물체에 대해서만 계산한다. 누락 물체를 포함한 전체 위치 정확도가 아니며 mm KPI도 아니다.
MVTec 13종을 나사·너트 두 클래스로 매핑한 실제 분류 정확도는 측정하지 않았다.
합성 형상의 비율 분류 테스트 통과와 실제 사진 분류 성능은 구분한다.

### 시각 검토 도구 추가

`scripts/preview_detections.py`로 보유 이미지 위에 회전 박스·중심·장축을 표시한다.
초록 박스는 비율 규칙상 screw, 주황 박스는 nut, 빨간 점은 중심, 자홍 선은 장축이다. 정답 표시는 아니다.
첫 6장 확인에서 붙은 물체가 하나의 큰 박스로 합쳐지며 작은 물체 누락과 배경 오검출이 관찰됐다.
현재 단순 이진화의 한계이며 개선 완료로 표시하지 않는다.

```bash
source ~/vision-arm/env.sh
PYTHONPATH=src/vision_pick_place python scripts/preview_detections.py \
  --images data/mvtec_screws/images --out logs/team2_detection_preview.jpg
PYTHONPATH=src/vision_pick_place python scripts/evaluate_detector.py \
  --annotations data/mvtec_screws/mvtec_screws_test.json \
  --images data/mvtec_screws/images --threshold 100 \
  --out logs/team2_detector_baseline.json
```

## 추가 시뮬레이션 시험

ROS_DOMAIN_ID=73, GZ_PARTITION=vision_arm_team2_check로 기존 세션과 격리해 실행했다.
`check_sim_week2.py`의 두 번째 실행에서 늦은 스폰 → attached → detached → attached → detached 성공.
붙인 물체가 약 0.09593m 상승한 후 분리·낙하했다. 이는 연결 메커니즘 시험이며 실제 접촉 집기·자동 분류 검증은 아니다.

## 저장된 증거와 남은 작업

실행 산출물은 git에서 제외되는 logs에 저장했다.

- `logs/team2_scene_check.json`: 실행 중 장면 구성·좌표
- `logs/team2_validation.json`: validation 설정 비교
- `logs/team2_detector_baseline.json`: test 평가
- `logs/team2_detection_preview.jpg`: 검출 시각화
- `logs/week2_sim_check.json`: 지정 이동·흡착 시험

남은 일:

- [ ] 프로젝트 GUI에서 매트·상자·패드 표시 및 RViz Plan·Execute 직접 확인.
- [ ] 실제 스마트폰 촬영본으로 나사·너트 분류와 실측 mm·각도 검증 (보정 담당과 협업).
- [ ] 그림자·나무 무늬 오검출 및 접촉 물체 분리 개선 후 validation/test 분리 평가.
- [x] 지정된 빈 공간의 두 자세 A↔B 5회 왕복 검증 (2026-11-17, 아래 상세).
- [ ] 임의의 RViz 목표, 실제 상자 투입, 물체를 붙인 이동·물리 충돌의 반복 안정성 검증.

코드·자동 재현 시험은 진행했지만 실제 촬영 자료와 화면 확인이 필요한 항목을 전체 완료로 판정하지 않는다.

## 3주차 팀 간 연결 구현 (2026-11-16)

미구현 TCP 브리지·서버를 구현해 검출 결과를 /robot/targets와 /robot/log로 연결했다.
보유 이미지용 통신 통합 런치를 추가했다. 타 팀 GUI 수정은 사용자 요청으로 원복했으며, 통합 런치도 GUI를 실행하지 않는다.
전체 테스트 65개 통과 및 실제 ROS/TCP 연결 시험 성공. GUI 수정본으로 진행했던 Qt 시험은 현재 버전에 적용되지 않는다. 검출·보정 코드는 다른 팀원의 최신 구현을 유지했다.
상세 규약·실행법·검증 제한은 [WEEK3.md](WEEK3.md)를 참고한다. 자동 집기 전체 사이클은 미구현이다.

사용자 범위 확인: 타 팀 파일은 수정하지 않음. 팀원 2 전용 tcp.launch.py로 통신만 실행하고 기존 비전 노드와 별도 연동 시험 통과. 상세 연결 규격은 WEEK3.md 마지막 절 참고.

### 3주차 부족 항목 보완 (2026-11-16)

- 오류가 섞인 성공 판정, 기본 100회 전체 경로 측정, 실패 JSON 저장을 수정.
- 실제 서버 재시작 시 연결 종료와 ROS 구독자 discovery 전 첫 타깃 유실 문제를 보완.
- 74개 테스트 모두 통과. 파일 이미지 100회 ACK·타깃 전달 각 100%, 오류 0, P95 16.17ms.
- 합성 보정/미보정 100회 및 실제 서버 재시작 복구 성공. 실제 보정 정확도·자동 집기 검증은 아님.
- 다른 팀원 담당 파일 변경 없음. 상세 사용법·증거·남은 항목은 WEEK3.md의 마지막 절 참고.

### 3주차 추가 보완: 시계·단계별 로그·연속 시험 (2026-11-16)

- 브리지 관찰 구간의 시계 변화 시 프레임→ACK 지연을 무효화하고 ACK 성공과 구분한다. 단조 시계 TCP 왕복 시간 추가.
- 기존 JSON/CSV 필드를 유지하면서 검증·연결·sendall 완료·ACK 단계를 추가 기록.
- 전체 103개 테스트 통과. 실제 이미지 100회 전달·ACK 모두 성공, 시계 경고 0. 프레임→ACK P95 19.74ms / TCP 왕복 P95 0.64ms.
- 연속 송수신 시간과 서버 재시작 횟수를 지정할 수 있는 시험 도구 추가. 결과 및 한계는 WEEK3.md 마지막 절 참고.
- 실제 영상·GUI·보정·로봇 담당 코드는 수정하지 않음. 이번 보완은 로컬 수정이며 커밋·푸시 별도.
- 3분 연속 시험 완료: 3,371개 패킷 ACK·타깃 전달 모두 성공, 오류·시계 경고 0. 이후 서버 재시작 5/5 복구. 시험용 합성 패킷 결과이며 실제 폰 영상·자동 집기를 검증한 것은 아니다.

## 상자 배치 조정 및 이동 검증 (2026-11-17) — 앞쪽 배치 시험, 이후 원복

사용자 요청으로 상자 배치에 필요한 `worlds/workcell.sdf`, `config/sim.yaml`만 변경했다. 통신·영상·GUI·보정·로봇 제어 코드는 수정하지 않았다.

- 기존 상자 중심 (0, ±0.55)m에서 작업 매트 양옆의 앞쪽 (0.45, ±0.30)m으로 이동. 순서는 screw(+Y), nut(-Y).
- 바깥 크기 0.15×0.15m, 벽 높이 0.06m, drop 높이 0.15m는 유지했다.
- 매트 표시 영역 X=[0.30,0.597], Y=[-0.15,0.06]m과 겹치지 않는다. 매트와 상자 외곽 사이 Y 간격은 screw 0.165m, nut 0.075m.
- SDF 상자 중심과 bin_x/bin_y 일치 확인. 기존 SDF 기반 MoveIt 충돌 박스 11개 유지, 충돌 검사 해제 없음.
- 별도 ROS_DOMAIN_ID=104 / GZ_PARTITION에서 headless Gazebo 실행. 기존 사용자 시뮬레이터에는 동작 명령을 보내지 않음.
- 작업점 위(0.40,-0.05,0.30)m → screw 접근(0.45,0.30,0.30) → drop 높이 0.15 → 상승 → 작업점 → nut 접근(0.45,-0.30,0.30) → drop 높이 0.15 → 상승 → 작업점의 9단계 이동 성공. 높이는 패드 끝 기준이며 tool0에는 0.05m 추가.
- 시험 속도 비율은 런타임 0.15. 일부 계획은 2~3회 시도 후 성공했다. 모든 자세·각도·속도에 대한 반복 안정성을 입증한 것은 아니다.
- 각 단계 성공 뒤 시뮬레이션 TF tool0 위치와 목표를 비교해 오차 약 0.46~0.95mm. 실측 정확도나 물체 투입 성공률이 아니다.
- 증거: `logs/bin_layout_check.json` (git 제외).
- 물체는 스폰하지 않았으며 접근·하강·복귀만 검증. 실제 흡착 물체의 간섭·낙하·상자 안 정착과 자동 집기 전체 사이클은 추가 검증 필요.
- 재확인 시 기존 실행을 Ctrl+C로 종료하고 `source ~/vision-arm/env.sh` 후 `ros2 launch ur5e_pick_place sim.launch.py` 실행.

이번 배치 변경은 로컬 저장 상태이며 아직 커밋·푸시하지 않았다.

### 사용자 배치 정정 (2026-11-17)

앞쪽 배치는 사용자 의도와 달라 원복했다. 현재 실제 상자와 설정 목적지는 screw (0, +0.55, 0)m, nut (0, -0.55, 0)m으로 로봇 좌우에 배치한다. 크기는 유지하며 중심 간 간격은 1.10m다. 위 앞쪽 배치의 9단계 시험 결과는 현재 배치의 검증 근거로 사용하지 않는다. 현재 좌우 배치에서의 반복 접근·투입은 별도 검증 대상이다.

### 좌우 간격 추가 확대 (2026-11-17)

사용자 요청의 20cm를 각 상자 바깥쪽 이동량으로 적용해 Y=±0.55m에서 ±0.75m로 변경했다. X=0과 상자 크기는 유지하고 SDF와 bin_y를 일치시켰다. 상자 중심 간격은 1.50m다. 위 ±0.55m 기록은 이전 상태이며, ±0.75m에서의 접근·투입 자세 도달성은 아직 검증하지 않았다.

### Gazebo/RViz 장면 일치 보완 (2026-11-17)

사용자가 Gazebo와 RViz 상자 위치가 다르게 보인다고 보고했다. 기존 실행의 실제 서버 상태는 확인하지 못해 과거 원인을 확정하지 않았다.
`sim.launch.py`에서 Gazebo와 pick_place_node에 같은 절대 world_file을 명시적으로 전달하고, GZ_PARTITION 미설정 시 실행마다 새 partition을 사용하도록 변경했다. 기존 GUI·서버의 기본 transport와 새 실행이 섞이는 것을 줄인다. GZ_PARTITION을 직접 설정했다면 해당 값을 유지하며, gz_partition 런치 인자로도 지정할 수 있다.

- 실제 Gazebo GUI 실행 (검증용 ROS_DOMAIN_ID=106) 후 /world/workcell/scene/info와 /get_planning_scene 직접 조회.
- screw_bin 실제 중심 (0, +0.75, 0)m, nut_bin (0, -0.75, 0)m 확인.
- 두 상자의 Gazebo 모델 중심 및 MoveIt 바닥·벽 위치/크기가 SDF와 모두 일치. 결과 ok=true.
- `scripts/check_workcell_layout.py` 추가. 최신 실행의 logs/workcell_session.json에서 월드·partition·ROS domain을 읽어 장면을 비교한다. 결과는 logs/workcell_layout_check.json.
- 이 검사는 위치·충돌 박스 일치만 확인하며 ±0.75m에서 실제 투입 동작 성공을 입증하지 않는다.

재실행: 기존 프로젝트 실행을 Ctrl+C로 종료하고 남은 이전 GUI 창을 닫는다. 새 터미널에서:

```bash
source ~/vision-arm/env.sh
unset GZ_PARTITION
ros2 launch ur5e_pick_place sim.launch.py
```

별도 터미널에서:

```bash
source ~/vision-arm/env.sh
python scripts/check_workcell_layout.py
```

직접 gz CLI나 check_sim_week2.py를 실행할 경우에는 같은 Gazebo partition을 사용해야 한다. 새 터미널에서 다음으로 최신 세션 partition을 설정한 뒤 실행한다:

```bash
export GZ_PARTITION="$(python -c 'import json; print(json.load(open("logs/workcell_session.json"))["gz_partition"])')"
```

이전 실행의 프로세스를 강제로 일괄 종료하지 않았으며, 검사 중 띄운 세션만 종료했다. 다른 팀원 GUI·영상·보정·로봇 제어 코드는 수정하지 않았다. 배치와 sim.launch.py 변경은 사용자 요청에 따른 것이며 아직 커밋하지 않았다.

### 좌우 추가 이동 (2026-11-17)

사용자 요청으로 두 상자를 각각 바깥으로 0.10m 더 이동했다. 현재 중심은 screw (0,+0.85,0)m, nut (0,-0.85,0)m이며 중심 간격 1.70m다. 크기는 유지하고 SDF·bin_y를 일치시켰다. 이전 ±0.75m 장면 일치 검증은 이전 상태의 기록이다. ±0.85m에서 로봇의 투입 자세 도달성은 미검증이며, 멀어진 위치 때문에 도달이 어려울 수 있다.

### 최신 팀원 3 코드와 연동 (2026-11-17)

- 팀원 2 전용 team2_link.launch.py로 기존 TCP 서버·브리지 및 수정하지 않은 기존 스폰 모듈을 같은 Gazebo partition에서 실행하도록 연결.
- 실제 Gazebo에서 미보정 거부, 합성 보정 패킷 TCP/타깃 동일성, 나사·너트 생성·재생성·삭제 확인. 실제 카메라 보정 정확도·자동 집기는 미검증.
- 전체 테스트 113개 중 112개 통과, 1개 실패: 다른 팀원의 test_workcell_scene.py가 옛 상자 Y=±0.45m를 기대하지만 사용자 배치는 ±0.85m. 테스트를 임의 수정·skip하지 않고 담당자 확인 대상으로 남김.
- 상세 실행 순서·오류별 설명은 WEEK3.md 마지막 절. 타 팀원 영상·보정·GUI·스폰·로봇 제어 원본은 수정하지 않음.

## 현재 배치의 반복 이동 검증 (2026-11-17)

상자 벽과 패드 충돌로 RViz 임의 목표가 실패한 이력에 대해, 충돌 검사를 유지한 상태에서 도달 가능한 빈 공간의 두 자세를 선정해 검증했다. 로봇 제어 원본은 수정하지 않고 새 검증 스크립트만 추가했다.

| 항목 | 결과 |
|---|---|
| 장면 | 상자 Y=±0.85m, SDF/Gazebo/MoveIt 장면 일치 검사 통과 |
| A | work_plane (100,100)mm, 패드 높이 0.30m → tool0 base_link (0.40,-0.05,0.35)m |
| B | work_plane (130,100)mm, 패드 높이 0.30m → tool0 base_link (0.43,-0.05,0.35)m |
| 자세·속도 | 툴 아래 방향 yaw=0°, 런타임 속도 비율 0.10 |
| 초기 A 이동 | 첫 계획 실패 후 두 번째 시도에서 성공. 초기 진입 항상 한 번에 성공한다고 판정하지 않음 |
| 왕복 | A→B→A 5회, 10개 이동 모두 첫 시도 계획·실행 성공 |
| 실제 위치 | 각 단계 완료 후 시뮬레이션 TF tool0 위치를 목표와 비교, 약 0.50~0.99mm 오차 |
| 설정 복원 | 시험 전 target·높이·속도·타임아웃 런타임 파라미터 복원 성공 |
| 증거 | logs/team2_repeat_motion_check.json (git 제외) |

Gazebo·RViz를 켠 별도 세션 ROS_DOMAIN_ID=109에서 기존 서비스로 이동을 실행했다. RViz 버튼을 사람이 눌렀다는 검증은 아니며, 같은 MoveIt 계획·실행/컨트롤러 경로를 자동 시험했다. ±0.85m 상자 접근·물체 투입·자동 집기나 임의 목표의 안정성 검증은 아니다.

### 같은 시험을 화면으로 재현

기존 프로젝트 실행은 종료하고 터미널 1에서:

```bash
source ~/vision-arm/env.sh
unset GZ_PARTITION
ros2 launch ur5e_pick_place sim.launch.py
```

터미널 2에서 (실제로 시뮬레이션 팔이 움직임):

```bash
source ~/vision-arm/env.sh
python scripts/check_team2_repeat_motion.py --execute --cycles 5
```

실행 중 RViz로 다른 이동을 동시에 요청하지 않는다. 실패하면 검사는 추가 이동 요청을 중단하고 JSON에 이유를 저장한다. 스폰·흡착은 하지 않는다. 실제 하드웨어에는 사용하지 않는다.
RViz 수동 조작에서는 목표를 현재 자세로 갱신한 뒤 이와 같은 빈 공간에서 작은 이동으로 시작한다. 충돌하는 목표가 실패하는 것은 유지해야 하는 정상 동작이며 충돌 검사를 끄지 않는다.

새 검증 도구와 문서 변경은 아직 커밋·푸시하지 않았다.

## 최신 진공 컵 코드와 팀원 2 연결 보완 — 2026-11-17

이번 변경은 팀원 2의 transport.py, 연결 런치·세션 모듈, 스폰 연결 검사와 통신 테스트에 한정했다. 팀원 3의 object_spawner.py·URDF와 팀원 1의 GUI·pick_place_node.py는 수정하지 않았다. 앞서 별도로 커밋한 박스 위치 테스트 수정도 유지했다.

### 수정 내용과 핵심 주석

- `transport.py`: 영상이 끊긴 사이 서버의 5초 유휴 제한으로 TCP가 닫히면 재개 첫 패킷이 실패했다. 마지막 ACK 이후 4초 이상 지났으면 **새 프레임을 보내기 전에** 기존 연결을 닫고 다시 연결한다. monotonic 시계를 사용한다. 이미 전송하고 ACK가 불명확한 패킷을 자동 재전송하지 않는 정책은 그대로다. 갑작스러운 서버 종료·네트워크 오류까지 모두 예방하는 것은 아니다.
- `test_transport.py`: 실제 서버에 패킷을 보낸 뒤 5.3초 쉬고 다음 패킷이 첫 시도에 ACK를 받는지 검사한다. 서버에 각 seq가 정확히 한 번 전달됐는지도 확인한다.
- `team2_session.py`·`team2_link.launch.py`: 동일 ROS 도메인·Gazebo partition을 사용하는 이유와, 최신 팀원 3 스폰 노드가 `/sim/vacuum`까지 제공한다는 연결 경계에 한국어 주석을 추가했다.
- `check_team2_spawn_link.py`: `/sim/vacuum` 존재를 확인한다. `--demo`는 전용 도메인에서 미보정 거부 → TCP/타깃 일치 → 스폰·재스폰 → 삭제 후 빈 장면 확인 → 진공 OFF 성공·빈 장면 ON 거부까지 검사한다. 조건을 우회하는 attach 토픽이나 팔 이동 명령은 보내지 않는다.

### 확인 결과

- 빌드: 2개 패키지 성공.
- 전체 테스트: **130개 통과, errors/failures/skipped 모두 0**. 박스 기대 좌표 ±0.85m 수정 포함.
- 합성 패킷 100개: ACK 100/100, 타깃 원본 일치 100/100. 서버 재시작 3회 복구, 실패 프레임 재전송 없음. `logs/team2_roundtrip_updated.json`.
- 최신 두 컵 URDF로 띄운 별도 headless Gazebo: 부품 2개 생성·재생성·삭제, 미보정 패킷 거부, `/sim/vacuum` OFF 성공·빈 장면 ON 거부 확인. `logs/team2_integration_updated/spawn.json`.
- 입력 전환 검사에서 기존 `server disconnected before ACK`는 사라졌다. 그러나 영상 노드를 새로 띄운 순간 ROS discovery 전에 발행된 seq 16·17은 브리지 기록에 없어서 최초 관측 100개 중 98개만 일치했다. `logs/team2_integration_updated/images.json`은 이 이유로 FAIL이며 덮어쓰지 않았다. 시작 직후부터 모든 프레임을 보존한다고 보장하지 않는다.

### 실행과 담당 간 경계

1. 터미널 1: `source ~/vision-arm/env.sh` 후 `ros2 launch ur5e_pick_place sim.launch.py`.
2. 터미널 2: 환경을 불러온 뒤 `ros2 launch vision_pick_place team2_link.launch.py`. 별도 tcp.launch.py 또는 스폰 노드를 중복 실행하지 않는다.
3. 영상 담당의 기존 vision_node를 실행한다. 보유 이미지 시험은 `ros2 run vision_pick_place vision_node --ros-args --params-file src/vision_pick_place/config/params_mvtec.yaml`.
4. ROS 노드가 서로 발견하고 실제 `/robot/log`에 ACK가 나오는지 확인한 뒤 `python scripts/check_team_integration.py`로 이후 100개를 검사한다. 준비 전 프레임까지 포함한 최초 시작 검사는 별도이며 위에서 실패로 기록했다.
5. 실제 스폰에는 실제 보정이 필요하다. MVTec 시험의 `calib_ok=false`를 강제로 true로 바꾸지 않는다. `--demo`는 격리된 nonzero ROS 도메인의 빈 시험 장면에서만 쓴다.

GUI ROS 워커는 여전히 다른 담당의 미구현 상태이며, 이번에 GUI 표시를 구현하거나 완료로 판정하지 않았다. `/sim/run_cycle`도 미구현이다. 실제 진공 흡착 이동·너트 링 컵의 12mm 오프셋을 적용한 자동 사이클·±0.85m 상자 투입·실제 폰 보정 정확도는 이번 검증 범위에 포함하지 않았다.

### 검사 노드 시작 시 관측 누락 구분

준비 후 실행한 검사에서도 `logs/team2_integration_ready/images.json`은 ACK 로그 8개(seq 64~71)를 관측하지 못해 FAIL이었다. 같은 실행의 브리지 CSV에는 해당 8개 모두 `acked=True`였고, 타깃도 100/100 일치했다. 이는 통신 실패보다 검사 노드의 `/robot/log` 연결이 다른 토픽보다 늦게 준비된 상황과 일치한다.

`check_team_integration.py --settle-s 2` 옵션을 추가했다. 모든 필수 토픽의 발행자 연결을 확인한 뒤 2초간 준비하고 이후 최초 100개를 검사한다. 기본값은 0이므로 기존 시작 직후 검사 동작은 유지한다. 제외한 준비 시간과 메시지 수·오류를 JSON의 `startup_excluded`, `settle_s`, `measurement_started_after_s`, `warmup_messages`, `warmup_errors`에 기록한다. 준비 구간을 제외한 통과를 시작 직후 무손실로 해석하면 안 된다.

최종 재검증: 최신 두 컵 URDF·MoveIt·팀원 3 스폰 노드와 팀원 2 연결 런치를 같은 격리 도메인(201)에서 실행했다. 스폰·재스폰·삭제·진공 서비스 검사는 통과했다. 보유 이미지 입력으로 `python scripts/check_team_integration.py --settle-s 2`를 실행한 결과, 토픽 준비 후 **100/100 ACK·타깃 일치**, 디버그 영상 수신, 오류·시계 경고 0이었다. frame-to-ACK P95 17.50ms, TCP 왕복 P95 0.70ms. 준비 시간은 시작 후 4.41초였고 제외 메시지 92개, 준비 구간에 관측된 오류 0개다. 증거: `logs/team2_integration_settled/images.json`, `spawn.json`. 검사에서 시작한 프로세스만 종료했고 기존 workcell_session.json은 복원했다.

이번 수정과 이 절은 아직 커밋·푸시하지 않았다.

## ±0.85m 상자 도달·실제 Gazebo 투입 확인 — 2026-11-17

사용자 요청으로 상자 위치는 유지한 채 별도의 headless Gazebo·MoveIt과 최신 두 컵 헤드를 실행했다. 다른 팀원 원본은 수정하지 않고 팀원 2 검사 도구 `scripts/check_team2_bin_reach.py`만 추가했다. 기존 미커밋 연결 보완도 보존했다.

### 결과

- 배치 확인: SDF·Gazebo·MoveIt 모두 screw_bin=(0,+0.85,0)m, nut_bin=(0,-0.85,0)m 일치. 충돌 물체 검사를 끄지 않았다.
- 빈 공구 도달: 컵 끝 높이 0.15m, tool0 높이 0.20m, yaw=0°로 양쪽 상자 중심까지 실제 이동 성공. TF 위치 오차 나사 상자 약 0.86mm, 너트 상자 약 0.89mm. 증거 `logs/team2_bin_reach_trial/reach.json`.
- 실제 투입 시험: 합성 보정 형식 패킷을 `/vision/detections` → TCP ACK → `/robot/targets`로 보낸 뒤 팀원 3 서비스로 나사·너트를 스폰했다. 실제 폰 보정 검증은 아니다.
- 나사: 5mm 컵으로 흡착 → 상승 → (0,+0.85)m 상자 위 15cm → 진공 OFF → 낙하·안정화. 최종 모델 위치 (0.000747,0.849736,0.005000)m, 상자 중심 대비 XY 오차 **0.79mm**.
- 너트: 링 컵으로 흡착 → 상승 → (0,-0.85)m 상자 위 15cm → 진공 OFF → 낙하·안정화. 최종 모델 위치 (0.000039,-0.850034,0.005000)m, 상자 중심 대비 XY 오차 **0.05mm**.
- 두 부품 모두 상자 바닥(z≈5mm)에 남았다. 놓은 뒤 3초 기다리고, 이어서 약 1초 동안 위치 10개를 받아 상자 내부 및 위치 변화 5mm 미만을 확인했다. 각 부품 1회 성공이다. 반복 성공률·실물 로봇 정밀도를 뜻하지 않는다.
- 너트 링 컵은 tool0 +X 12mm이므로 yaw=0°에서 집기·투입 공구 목표를 부품 목표보다 X=-12mm 보정했다. 이때 너트 투입 tool0 목표는 (-0.012,-0.85,0.20)m다.
- 실제 투입 실행의 팔 이동 11단계 모두 성공. 첫 작업 영역 이동은 내부 계획 3회째 성공했고 이후 상자·집기·투입 이동은 모두 1회째 성공했다. 검사 속도 비율 0.10, 계획 시간 10초/회, 계획 최대 3회였다. 모든 임시 파라미터는 종료 때 복원했다.
- 최종 증거: `logs/team2_bin_drop_retry/reach.json` (`ok=true`, `actual_part_drop_verified=true`, `parameters_restored=true`), 같은 폴더의 `layout.json`, `reach.log`, `sim.log`.

### 발견한 한계·실패 기록

첫 투입 시도(`logs/team2_bin_drop_trial/reach.json`)는 상자 목표를 시험하기 전, 기존 작업 영역 (0.4,-0.05,컵 끝 0.25)m로 가는 초기 계획이 3회 실패(code 99999)해 중단했다. 상자 도달 불가로 판정하지 않았다. 다음 시도에서 검사에 한해 계획 시간을 5초→10초로 늘려 완료했다. `pick_place_node.py`나 영구 설정은 바꾸지 않았다. 따라서 초기 OMPL 계획 안정성 문제까지 해결됐다고 보지 않는다.

현재 위치는 **이 시뮬레이션·공구·높이·자세에서 실제 투입 가능**으로 확인했다. 더 낮은 높이로 상자 안까지 하강하거나 다른 공구 자세로 접근하는 모든 경우를 보장하지 않는다. `/sim/run_cycle` 구현 완료도 아니다. Gazebo의 흡착 기하 판정과 고정 연결을 사용한 결과이며 실제 진공 누설·접촉 물리는 별도다.

### 다시 확인하는 방법

기존 작업과 분리한 비어 있는 시험 도메인을 사용한다. 아래 73은 다른 프로그램이 사용하지 않을 때의 예시다. `--drop`은 나사·너트를 생성하고 시뮬레이션 팔을 자동으로 움직인다.

터미널 1:

```bash
source ~/vision-arm/env.sh
export ROS_DOMAIN_ID=73
unset GZ_PARTITION
ros2 launch ur5e_pick_place sim.launch.py
```

터미널 2:

```bash
source ~/vision-arm/env.sh
export ROS_DOMAIN_ID=73
ros2 launch vision_pick_place team2_link.launch.py
```

터미널 3:

```bash
source ~/vision-arm/env.sh
export ROS_DOMAIN_ID=73
python scripts/check_team2_bin_reach.py --execute --drop
```

상자 위치를 바꾸지 않고 빈 공구 도달만 검사하려면 `--drop`을 빼고 실행한다. 기본 결과는 `logs/team2_bin_reach_check.json`에 저장된다. 실패 시 JSON의 `error`·각 단계 `message`를 확인한다. 컨트롤러 실패나 응답 시간 초과는 추가 이동 없이 중단한다.

이번 새 검사 도구와 기록은 아직 커밋·푸시하지 않았다.

## Gazebo·RViz 화면 시연 — 2026-11-17

사용자 요청으로 GUI가 있는 별도 시연 스택(ROS_DOMAIN_ID=205, GZ_PARTITION=vision_arm_aaec475d4e09)을 실행했다. GUI 프로세스는 실행 확인했으며 화면을 사용자가 직접 봤다는 확인은 아직 받지 않았다. 종료하지 않고 상자와 부품을 볼 수 있도록 열어 두었다.

- 첫 GUI 시연: 나사 투입 성공 뒤 너트를 집으러 가는 낮은 접근 목표의 계획이 3회 실패했다. `logs/team2_gui_demo/reach.json`은 실패 그대로 보존했다.
- 팀원 2 검사에 다음 부품으로 넘어갈 때의 높은 안쪽 경유점 (0.20, ±0.55, 컵 끝 0.25)m을 추가하고 같은 스택에서 재시연했다. 다른 담당 원본은 수정하지 않았다.
- 재시연: 나사 투입 성공 → 안쪽 경유점 성공 → 너트 접근·흡착·투입 성공. 나사 최종 (0.000846,0.850523,0.005000)m, 너트 최종 (-0.000285,-0.849400,0.005000)m. 둘 다 상자 안에서 안정화했다.
- 다만 마지막 너트 투입 뒤 빈 공구 대기점 복귀에서 계획 실패(code 99999, 3회)가 발생했다. 따라서 재시연 전체 결과 `logs/team2_gui_demo/retry.json`은 `ok=false`이며 이를 통과로 바꾸지 않았다. 두 `drops[].success`는 true, 임시 파라미터 복원도 true다. 부품은 상자 안에 있고 팔은 정지한 상태다.
- 현재 검사 도구는 경유점을 **다음 부품이 있을 때만** 사용하도록 정리했다. 마지막 투입 이후 빈 공구 복귀는 별도 검사로 남긴다. 완료 전 JSON은 `status=running`·`ok=false`로 유지하도록 정리했다. 이 마지막 제어 흐름 정리는 문법 검사만 했으며 전체 GUI 시연을 다시 반복하지 않았다.
- GUI 실행에서의 계획 재시도·복귀 실패까지 해결된 것으로 기록하지 않는다. 실물 로봇 시험도 아니다.
