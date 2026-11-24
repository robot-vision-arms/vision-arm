# 팀원 2 (성민) — 실행·통합 검증 기록

> 최신 검증(2026-11-23 추가): 새 배치 3회·27/27 성공, 통신 600/600, 최종 조건 9개·126.92초. 전체 테스트 209개 통과. native 종료 오류·조건별 시간 초과는 남아 있다. [최신 보완·실행 방법](#team2-20261123-stability)

초기 확인일: 2026-11-13. 실행 위치: `~/vision-arm`.
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

## 팀원 2 연결 사전 검사 보완 — 2026-11-19

검증 기준은 현재 작업 트리(HEAD `4fa2de4` + 팀원 2 수정본)다. 원격의 모델 비교 추가 커밋을 자동 병합하지 않았다. 기존 주석 수정(robot_bridge.py, robot_server.py, preview_detections.py)은 보존했다. 다른 담당의 camera_stream.py·vision_node.py·보정·GUI·object_spawner.py·pick_place_node.py·URDF·공유 설정 파일은 수정하지 않았다.

### 연결 경계에서 바꾼 것

- `team2_session.py`: 파일 설정과 실행 중 `/pick_place_node/get_parameters` 응답을 비교한다. world_name, 작업 평면 X/Y/Z·yaw, 공구 길이, 슬롯 개수, 종류 배열 순서, use_sim_time이 다르면 새 연결 노드를 시작하지 않는다. 같은 월드 이름만으로 잘못된 mm→로봇 좌표 변환이나 공구 높이를 허용하지 않도록 했다.
- 중복 TCP·스폰 노드 및 실제 스폰 서비스의 존재를 확인한다. 검사 노드의 ROS 발견 시간을 고려해 서비스 서버 발견을 기다린다. 서비스 이름 목록에는 클라이언트만 만든 이름도 나타나므로, 이름 목록만으로 중복을 판정하지 않는다. 기존 노드는 종료·수정하지 않는다. 동시에 여러 런치를 시작하는 경쟁 상황까지 원자적으로 막는 장치는 아니다.
- `team2_link.launch.py`: 위 검사를 거쳐 기존 팀원 3 노드를 그대로 실행한다. 포트 범위와 로컬 TCP 포트 사용 여부도 확인한다. 다른 ROS 도메인의 서버에 잘못 접속하는 경우를 줄인다.
- Gazebo 응답은 비어 있지 않은 문자열이라는 이유만으로 통과시키지 않고 Scene protobuf로 파싱한다. 실제 Harmonic 응답이 월드 name을 생략할 수 있으므로 모델·조명 내용이 있는 정상 응답을 허용하고, 명시된 이름이 다른 경우와 빈 장면·오류 문구는 거부한다.
- 핵심 판단 이유는 해당 연결 코드에 한국어 주석으로 남겼다. 이전 통신 모듈·미리보기 주석도 유지했다.

### 검증

- 빌드: 2개 패키지 성공.
- 전체 테스트: **150개 통과, 오류·실패·skip 0**. Protobuf 등의 폐기 예정 경고 3개가 있으며 테스트 실패는 아니다. 새 회귀 테스트는 좌표·공구 길이·종류 순서·시간 설정 불일치, 누락된 세션·설정, 정상/잘못된 Gazebo 응답을 포함한다.
- 격리된 ROS_DOMAIN_ID=208의 headless Gazebo·MoveIt + team2_link 실행. 팔 이동 명령은 보내지 않았다.
- 다른 sim_params의 work_plane_x=0.31m와 실행 중 로봇의 0.30m 불일치를 정확히 거부했다. 정상 설정에서는 시작 성공.
- 다른 TCP 포트를 지정해 연결 런치를 한 번 더 시작해도, 이미 실행 중인 TCP·스폰 스택을 감지해 거부했다. 정상 스택은 유지됐다.
- 합성 미보정 패킷 스폰 거부, 보정 형식 합성 패킷의 ACK·타깃 동일성, 나사·너트 2개 스폰·재스폰·삭제, 진공 OFF·빈 장면 ON 거부 확인.
- 기존 팀원 vision_node를 MVTec 설정으로 실행 → 연결 준비 후 **100/100 ACK·원본 타깃 일치**, 디버그 영상 수신. 오류·시계 경고 0. frame-to-ACK P95 약 23.54ms, TCP 왕복 P95 약 1.16ms. `--settle-s 2`를 사용했으므로 시작 직후 구간은 제외했고 준비 중 관측 오류도 0이었다.
- 최종 근거: `logs/team2_link_review_20261119_verified/summary.json`, `images.json`, `spawn.json`, `reject_mismatch.log`, `reject_duplicate.log`. 앞선 검사 코드 보완 중의 실패 기록도 별도 폴더에 보존했다. 시험에서 시작한 프로세스만 종료했고 기존 workcell_session.json을 복원했다.

실행 순서는 기존처럼 sim.launch.py → team2_link.launch.py → 영상 담당 vision_node다. 별도 tcp.launch.py나 스폰 노드를 중복 실행하면 이제 오류 메시지와 함께 시작을 거부하므로, 기존 연결 런치를 사용하거나 그 런치를 먼저 종료한 뒤 다시 실행한다. 월드·좌표·공구 설정이 다르면 현재 시뮬레이터와 같은 sim_params를 사용해야 한다.

실제 폰 보정의 정확도, 미구현 GUI 워커, `/sim/run_cycle` 전체 사이클과 기존 팔 복귀 계획 실패는 이번에 구현·검증한 것으로 기록하지 않는다. 이번 시험은 팔을 움직이지 않았다. 이번 변경은 아직 커밋·푸시하지 않았다.

<a id="team2-20261120-integration"></a>

## 2026-11-20 팀원 2 통신 누락·기존 물체 집기 보완

이 절은 위의 날짜별 과거 기록 이후의 변경이다. `e53b472` 기반 작업 트리에서 보완했다. 다른 담당 원본인 `vision_node.py`, `object_spawner.py`, `pick_place_node.py`, GUI, 모델·보정 파일은 수정하지 않았다. 통신 코드와 새 팀원 2 어댑터·검사 도구, 실행 연결만 수정했다.

### 통신 누락 원인과 보완

- 기존 브리지는 ROS 수신 콜백에서 TCP ACK를 기다렸고 DDS 수신 깊이는 1이었다. ACK 처리에 60 ms 지연을 주고 20 ms 간격으로 30개를 발행하는 비교에서 관찰자는 전부 받았지만 기존 브리지는 11개만 처리했다. 수정 후에는 같은 조건에서 30개 전부 처리했다. 이는 기존 유실 가능 구조를 재현한 것이며 과거 PC 스케줄링 지연의 정확한 원인까지 추정으로 확정하지 않는다.
- `robot_bridge.py`: 수신 콜백과 단일 TCP 송신 워커를 분리했다. 수신·작업 큐 깊이는 32이며 무한 적재하지 않는다. 작업 큐에서 2초 넘게 대기한 패킷과 큐 초과로 밀려난 패킷은 전송하지 않고 `queue_expired`·`queue_overflow` 실패로 기록한다. `queue_wait_ms`, `processing_ms`도 CSV에 남긴다. ACK가 불확실한 패킷의 자동 재전송은 추가하지 않았다.
- 검사기의 시작 경계 문제도 확인했다. 준비 구간에 ACK·목표를 먼저 관찰하고 같은 번호의 원본이 측정 시작 뒤 도착하면 가짜 누락이 됐다. `check_team_integration.py`는 준비 구간의 모든 토픽에서 본 번호를 보존하고 그 번호는 늦게 도착해도 측정 표본에서 제외한다. 대신 새 번호 100개를 측정하고 제외 수는 `boundary_excluded_source_packets`로 공개한다. `--settle-s 0`의 시작 포함 측정은 유지한다. ACK 선도착을 재현하는 회귀 테스트 2개를 추가했다.

### 기존 물체를 재생성하지 않는 집기

- 새 `team2_pick_place_node.py`가 기존 이동·흡착 구현을 상속한다. 기본 `cycle_scene_mode=existing`에서는 `/sim/spawn_objects`를 부르지 않는다.
- 영상 검출 순서와 Gazebo 슬롯 번호는 다를 수 있다. 기존 TF 위치에 5 mm 이내로 유일하게 대응되는 물체를 고른다. 마커의 모델 형상으로 확인한 종류와 검출 종류가 다르거나, 대응이 없거나 여러 개이거나, 같은 물체에 검출이 중복 대응되면 이동 전에 거부한다. 틀린 분류에 맞춰 Gazebo 모델을 새로 만들지 않는다.
- 보완 중 무작위 장면에서 상자 이동 계획 실패 및 투입 후 복귀 자기 충돌이 재현됐다. 실패 기록은 `logs/team2_completion_20261120/live`, `waypoint`, `inset`, `return_route`에 보존했다.
- 상자 위치 ±0.85 m는 유지한다. 상자 접근은 안쪽 높은 경유점을 사용하고 놓기 목표는 중심보다 로봇 쪽으로 35 mm 이동한다. 안착 판정 기준은 원래 상자 중심이다. 복귀는 안쪽 높은 경유점과 로봇 정면을 거친다. 충돌 검사를 끄지 않았다.
- 설치된 pymoveit2의 `timeout_sec`는 IK 응답 대기 제한이며 IK 요청의 서버 계산 시간과 별개다. 어댑터는 `/compute_ik` 요청에 서버 계산 1초, 링크 `tool0`, 충돌 검사 사용을 명시하고 현재 관절과 대기 관절을 초기값으로 사용한다.
- 마지막 복귀가 경고로 끝나도 집기 전체를 성공으로 오해하지 않도록 대기 관절값 도달 여부도 확인한다.
- 사진 좌표를 Gazebo로 재현하는 기존 모드는 `cycle_scene_mode:=respawn`으로 명시할 때 사용할 수 있다. 그 모드의 결과를 기존 물체 집기 검증과 섞지 않는다.

### 실행 방법

기존 노드는 파일 수정만으로 갱신되지 않는다. 기존 sim·link·vision 실행을 종료한 뒤 프로젝트 루트에서 재빌드하고 각 터미널에서 `source env.sh`를 한다.

```bash
colcon build --symlink-install
# 터미널 1
ros2 launch ur5e_pick_place sim.launch.py cycle_scene_mode:=existing
# 터미널 2
ros2 launch vision_pick_place team2_link.launch.py
# 터미널 3
ros2 run vision_pick_place vision_node --ros-args --params-file src/vision_pick_place/config/params_sim.yaml
# 터미널 4: 팔이 정지한 상태에서 실행. 기존 obj_k를 교체하고 시뮬레이션 팔을 움직인다.
python scripts/check_team2_existing_cycles.py --execute --cycles 7 --transport-trials 3
```

검사 도구는 매 회 `/sim/spawn_random`으로 시험 장면만 초기화한다. 이후 실제 가상 카메라와 TCP를 거쳐 인식하고 기존 물체를 집는다. 합성 검출 패킷은 발행하지 않는다. 집기 전후 Gazebo 모델 ID가 동일하고, 3개 투입·대기 자세 복귀가 확인돼야 한 회 통과다. 실패하면 재시도로 덮지 않고 중단한다. 결과는 기본 `logs/team2_existing_cycles.json`과 같은 폴더의 회차별 통신 JSON에 남긴다.

### 검증 범위

- 두 패키지 빌드 성공. 수신 워커·큐 만료·초과, 위치 대응·분류 불일치·중복 대응·복귀 확인 및 검사 시작 경계를 회귀 테스트에 포함했다.
- 실제 가상 카메라 통신 100개씩 3회: 모두 ACK·목표 일치 100/100. frame-to-ACK P95 53.43 / 68.49 / 53.21 ms. 최초 보완 단계에서 측정한 값이며 뒤의 이동 보완과 구분한다.
- 로컬 MVTec 실사 이미지 384장의 재생 입력: 측정 구간 ACK·목표 일치 100/100, P95 약 62.87 ms. 미보정(`calib_ok=false`)이므로 mm 좌표·실물 집기 정확도 검증이 아니다. 준비·측정 경계 문제를 발견했던 최초 실패 기록도 별도 `mvtec` 폴더에 보존했다.
- 최종 이동 보완본: 한 세션에서 무작위 **7사이클·21개 전부 투입 판정 통과**, 전 회차 모델 ID 유지·대기 자세 복귀 확인. 회당 112.5~146.4초. 수동 좌표 이동과 자동 경유점을 분리한 최종 파일의 새 프로세스에서도 **1사이클·3개 추가 통과**했다. 총 8회·24개이며, 임의의 모든 배치에서의 영구적인 성공률을 보장하는 값은 아니다.
- 최종 전체 회귀 테스트: **189개 통과**, 두 패키지 빌드 통과. 새 프로세스 통신도 100/100 ACK·목표 일치다. 다만 해당 추가 측정의 frame-to-ACK P95는 **641.84 ms로 300 ms 목표를 초과**했다. TCP 왕복 P95 7.34 ms, 큰 지연 샘플의 브리지 큐 대기는 약 0.4~3.5 ms로 짧아 수신 이전 구간의 추가 분석이 필요하다. 통신 성공·측정 유효성과 지연 KPI 충족을 혼동하지 않는다.
- 남은 로그: Octomap 3D 센서 플러그인 미설정 메시지가 있다. 이번 시험은 등록된 정적 충돌 모델로 수행했다. 추가 시험 종료 정리 중 ros_gz_bridge와 move_group의 native 종료 코드 -11도 관찰됐다. 집기 중 실패와 구분하지만 정상 종료 안정성까지 통과로 처리하지 않는다.
- 실제 촬영한 작업면 사진은 이 PC의 captures/ground_truth에 없고 실물 로봇 시험도 하지 않았다. 보정 파일이 있다는 것만으로 실물 검증 성공으로 처리하지 않는다. 실물 사진이 준비되면 같은 촬영 조건의 보정·독립 정답으로 검증해야 한다.

근거 폴더: `logs/team2_completion_20261120/`. 로그는 로컬 자료이며 Git에 포함되지 않는다. 이 보완은 아직 커밋·푸시하지 않았다.

<a id="team2-20261123-demo"></a>
## 2026-11-23 — 10개 작업 셀·기존 물체 집기·실물형 외형

HEAD `7fe6754` 기준 팀원 2 확장이다. 사용자 선택은 **현재 치수로 외형 개선**이다. 다른 담당의 비전·GUI·스폰·이동 원본은 유지하고 팀원 2 어댑터와 실행 연결로 확장했다. 공유 실행 파일 `sim.launch.py`, `moveit.launch.py`에는 선택 설정·URDF 전달을 추가했다. 기존 기본 실행의 3개 설정과 원본 모델·보정 파일은 유지한다.

### 요청한 다섯 항목과 결과

| 항목 | 구현·실행 결과 |
|---|---|
| 10개 부품·슬롯 및 큰 상자 | 전용 Xacro로 슬롯 0~9 구성. 생성 프로필에서 상자 폭 15→22 cm, 중심 ±0.85 m 유지. GUI 개수 문구도 선택 개수를 반영 |
| 검출 좌표로 기존 물체 집기 | 실제 Gazebo 카메라→비전→TCP→기존 모델 대응→집기. 사이클 안에서 재생성하지 않으며 모델 ID 전후 일치 검사. 분류 불일치·좌표 중복은 이동 전에 거부 |
| 20회 이상 성공 | 기본 형상 10개씩 속도 0.3/0.5 두 회, **20/20 투입·대기 자세 복귀 통과**. 같은 시험 배치의 결과이며 모든 임의 배치의 성공률 보장은 아님 |
| 9개 2분 30초 | 최종 외형 개선·환경 무작위화·GUI 실행에서 **9/9, 123.17초**, 속도 0.5, 중간 대기 자세 생략. 이전 독립 실행도 130.21초. 시작·스폰·촬영 준비 시간은 제외하고 사이클과 사후 확인 시간을 측정 |
| 실물형 모델·데이터 장면 | 나사산·육각 머리·원형 구멍 육각 너트 추가. 크기·질량·충돌 형상 유지. 조명·바닥/매트 색·카메라 높이 무작위화, 실제 렌더링 ArUco 재보정, 이미지·실제 모델 자세 기반 라벨 저장 |

20개 비교의 시간은 327.81/230.76초였다. 이후 시험 실행기가 종료 시 남긴 Gazebo 서버가 CPU를 사용한 사실을 찾아 실행기 종료 처리를 수정했다. 따라서 이 두 시간은 깨끗한 성능 기준으로 사용하지 않는다. 최종 9개 측정은 남은 시험 서버를 정리한 후 실행했다. 종료 정리는 실행기가 직접 생성한 프로세스 그룹에만 적용한다.

### 수정한 핵심과 이유

- `team2_demo_profile.py`: 모델·월드·설정의 생성 복사본을 사용한다. 원본 데이터와 다른 팀원의 기본 실행에 영향을 주지 않는다. 충돌 형상은 기존 근사 형상이므로 나사산 접촉·체결 물리 검증은 아니다.
- `team2_pick_place_node.py`: 확대 상자의 실제 벽·바닥과 회전된 물체 전체 크기로 투입 여부를 판정한다. 기존 15 cm 기준의 거짓 실패만 보정하며 다른 집기 실패는 숨기지 않는다. 마지막 부품은 항상 대기 관절 복귀를 확인한다. TF 구독을 같은 실행기에서 처리해 별도 TF 스레드의 실행기 중복 사용 오류를 보완했다.
- `team2_shaft_pose.py`, `team2_shaft_vision.py`: 육각 머리 때문에 영상 외곽 중심이 축에서 약 1 mm 벗어나는 문제를 중간 축 영역으로 보정한다. 영상만 사용하며 Gazebo 정답 좌표를 검출에 주입하지 않는다. 원본 비전 파이프라인은 그대로 사용한다.
- `team2_scene_labels.py`, `capture_team2_scene.py`: 검출 예측을 정답으로 복사하지 않고 스폰 종류·실제 모델 TF·카메라 투영으로 위치·축각·경계 라벨을 만든다. 너트 각도는 null이다. 정지 장면 수집용이며 움직이는 물체의 정밀 시간 동기화 검증은 아니다.
- `team2_demo_gui.py`: 기존 GUI를 상속해 최대 개수·스폰 서비스·전체 장면 탭만 연결한다. 핵심 계산·안전 경계에는 주석을 추가했다.

10개 정지 장면(`final_dataset10`)에서는 실제 검출 10개, 종류 10/10 일치, 위치 평균 오차 0.144 mm, 나사 축각 평균 오차 0.066°였다. **한 Gazebo 장면의 합성 정답 비교**이며 실물 카메라 정확도나 학습 성능으로 해석하지 않는다. 여러 환경의 대규모 데이터 수집·학습은 아직 수행하지 않았다.

### 화면으로 실행

```bash
cd ~/vision-arm
source env.sh
colcon build --symlink-install
python scripts/run_team2_demo.py --gui --realistic --randomize --seed 3 --speed .5 --compact-return
```

한 명령이 Gazebo 서버·MoveIt·통신·비전·프로젝트 GUI를 연결한다. GUI에서 **부품 10개 배치 → 검출 10개 확인 → 자동 사이클** 순서로 사용한다. 종료는 실행 터미널에서 Ctrl+C. 기본 ROS 도메인 70·TCP 포트 19070이며 같은 도메인에 로봇이 있으면 거부한다. Gazebo/RViz 창도 필요하면 `--gazebo-gui --rviz`를 추가한다. 창을 추가하면 성능 수치가 달라질 수 있다.

9개 자동 시험은 위 명령에 `--objects 9 --cycles 1 --out logs/team2_bench9`를 추가한다. 촬영만 하려면 `--capture-only --out logs/team2_dataset/seed3`를 추가한다. `--seed`를 바꾸면 환경·부품 배치가 바뀐다. 출력에는 생성 설정, 개별 프로세스 로그, `summary.json`, 이미지와 라벨이 남는다. 이 자료는 `logs/` 아래이므로 Git에 자동 업로드되지 않는다.

### 검증 및 남은 제한

- 근거: `logs/team2_20261123/comparison_world_bounds/`(20개), `final_acceptance9/`(최종 9개), `final_dataset10/`(10개 자세 비교), `final_capture10/`(최종 GUI·수집). 개발 중 실패 로그도 보존했다.
- 최종 9개는 전 모델 ID 유지, 대기 관절 오차 최대 0.001 rad 미만, TF 스레드 예외 없음. **실물 로봇 시험은 하지 않았다.**
- 충돌 검사는 유지한다. 일부 계획이 충돌 검사에서 거부된 뒤 안전한 재계획으로 통과했다. Octomap 3D 센서 미설정 메시지, 없는 모델 초기 삭제 메시지, 종료 시 move_group native 코드 -11 및 Gazebo 강제 종료 단계는 남는다. 따라서 로그 오류 전체가 없어졌다고 판단하지 않는다. 초기 삭제 메시지를 피하려고 다른 담당 스폰 원본을 수정하지 않았다.
- 원래 통신 지연 KPI의 모든 조건을 이번 시험에서 다시 측정한 것은 아니다. 이전의 지연 초과 기록을 성공으로 변경하지 않는다.
- 사용자 요청으로 이번 구현·검증 기록을 커밋·푸시 대상으로 정리했다. 실행 로그·이미지는 Git 제외 경로인 `logs/`에 로컬 보관한다.

최종 확인: 두 패키지 빌드 성공, 전체 회귀 테스트 **203개 통과** (`tests_final.xml`). 최초 제한 환경에서는 소켓 생성이 차단되어 실패했고, 허용된 실행에서 확인했다. CSV 검사 중 초기 헤더가 버퍼에 남는 경쟁 조건도 발견해 팀원 2 브리지에서 헤더를 즉시 flush하도록 보완한 뒤 전체 재검증했다. 최종 GUI는 검출 행 10개·전체 장면 960×540 수신을 확인했다. 시험 도메인 192~202에 남은 Gazebo·ROS 서버가 없음을 확인했다.

<a id="team2-20261123-stability"></a>
## 2026-11-23 추가 — 반복 안정성·가속도·통신 지연 확인

앞 절의 커밋 `0b48711` 이후 추가 작업이다. 팀원 2 어댑터·프로필·실행기·GUI 확장·검사와 이 기록만 수정했다. 다른 담당 원본 12개(비전, GUI, 스폰, 이동, 기본 월드·모델·URDF·설정)가 HEAD와 동일함을 다시 검사했다.

### 오류 보완과 핵심 주석

1. **가속도 기본값 경고 보완:** 설치된 pymoveit2의 가속도 초기값은 0이고 기존 이동 준비는 속도만 설정했다. MoveIt은 0을 유효한 배율로 인정하지 않고 1로 대체했다. 팀원 2 `prepare_motion()`에서 `team2_acceleration_scaling`을 유한한 (0, 1] 값으로 검증한 다음 `moveit2.max_acceleration`에 전달한다. 기본 0.5, 실행기 `--acceleration`으로 명시 변경 가능하다. 잘못된 값은 이동 준비 전에 거부한다. 실제 시험 노드의 파라미터도 조회해 속도 0.6·가속도 0.5를 확인했다.
2. **GUI 종료 예외 보완:** 팀원 2 래퍼에서 SIGINT를 Qt 종료로 연결하고 `aboutToQuit`에서 기존 창 닫기·워커 정리를 호출한다. 다른 담당 GUI 원본은 그대로다. 이번 세 집기 실행에서 GUI `KeyboardInterrupt`/Traceback은 없었다. 통신 검사가 길어 초기 화면이 비어 있을 때를 위해 선택 개수 검출 후 화면을 한 번 더 저장한다.
3. **반복 배치 범위 명시:** 기존 seed는 종류·축각·환경만 변경하고 중심 위치는 고정이었다. `--position-jitter-mm`로 각 축 최대 ±5 mm 변동을 추가했다. 실제 시험은 ±4 mm다. 마커 여백·기존 간격을 지키는 제한된 변동이며 임의 밀집·겹침·모든 각도 검증이 아니다. 위치 변동은 초기 배치에만 쓰고 실제 집기는 카메라 검출 좌표를 사용한다.
4. **측정·결과 분리:** 실행기에 `--transport-trials`와 `--latency-limit-ms`를 추가했다. 합성 배치 패킷이 섞이지 않도록 초기화 전에 측정한다. 기존 `latency_ok`는 측정 유효성이므로, P95 300 ms 목표는 `latency_kpi_passed`로 별도 판정한다. 통신 실패도 원본 JSON을 보존한다. 9개 집기의 시간 목표도 기능 성공과 별도로 `cycle_time_kpi_passed`에 기록한다. 최상위 `ok`만 보고 시간 목표까지 통과했다고 판단하지 않는다.
5. **오류 기록 자동화:** `scripts/review_team2_runtime.py`는 자기 충돌, 거부된 계획, 가속도 경고, 초기 삭제·Octomap·native 종료 오류를 분리해 센다. 실행기 `summary.json`의 `runtime_log_review`에도 저장한다. 회복한 계획 오류를 숨기거나 집기 실패 수와 혼동하지 않도록 주석을 추가했다.

### 새 Gazebo 실행 결과

모두 외형 개선·환경 무작위화·프로젝트 GUI·기존 물체 집기 방식이다. 각 실행은 독립 ROS 도메인으로 순차 진행했다. GUI는 offscreen 렌더링·저장 화면으로 확인했으며 사람이 실제 창에서 버튼을 누른 시험과 구분한다.

| seed | 속도 / 가속도 | 위치 변동 | 투입 | 사이클 시간 | 150초 목표 |
|---|---|---|---|---|---|
| 3 | 0.5 / 0.5 | 없음 | 9/9 | 154.52초 | 초과 |
| 4 | 0.6 / 0.5 | 없음 | 9/9 | 158.26초 | 초과 |
| 5 | 0.6 / 0.7 | 각 축 ±4 mm | 9/9 | 126.92초 | 통과 |

총 **27/27 투입 성공**, 모든 회차 모델 ID 유지·대기 관절 복귀 확인. 세 실행 모두 자기 충돌 접촉·INVALID_MOTION_PLAN·가속도 기본값 경고가 없었다. 임의 모든 배치에서 해결됐다고 확정할 수는 없다. 서로 다른 seed는 환경·종류·각도가 달라 이 표를 속도만의 인과 비교로 사용하지 않는다. 앞 절의 123.17초는 가속도가 기본값 1로 대체되던 이전 조건이며 지금의 명시적 가속도 조건과 구분한다. 시험 시간은 시작·배치·촬영 준비를 제외한 사이클과 사후 확인이다.

통신은 총 **100개씩 6회·600/600 ACK 및 목표 일치**, 누락·시계 경고 없음. P95는 시작 전 22.87 / 21.23 / 20.60 / 22.42 / **297.74 ms**, seed 5의 팔 이동 중 별도 관찰은 **61.27 ms**였다. 모두 300 ms 이하지만 297.74 ms는 여유가 작아 항상 낮은 지연을 보장한다고 볼 수 없다. 이동 중 검사 JSON의 `robot_motion_commanded=false`는 관찰자가 로봇 명령을 보내지 않았다는 뜻이다. 실제 팔은 별도 실행기의 자동 사이클로 이동 중이었다.

최종 두 패키지 빌드 성공, 전체 **209개 테스트 통과**. 새 테스트는 잘못된 가속도 이동 전 거부, 유효 배율 전달, 재현 가능한 위치 변동·간격·범위 거부를 확인한다. 최종 설정의 집기 없는 GUI·촬영 재확인도 `gui_confirmation/`에 저장했다.

### 남은 문제와 완료 범위

- **종료 문제는 미해결:** 세 실행 모두 종료 시 move_group native 코드 -11, Gazebo SIGTERM 단계가 재현됐다. 정확한 native 충돌 원인은 미확정이다. Python 예외 보완으로 SDK 비정상 종료까지 해결됐다고 하지 않는다. 다른 담당 원본·시스템 ROS 패키지를 임의 변경하지 않았다.
- **Octomap 미설정·없는 모델 초기 삭제 메시지 유지:** 정적 충돌 모델을 사용한 시험이고, 초기 배치는 성공했다. 다른 담당 스폰 구현은 수정하지 않았다.
- **시험 범위 제한:** 현재 장면 3개와 작은 위치 변동에서 통과했다. 밀집·가림·겹침·모든 무작위 배치·실물 로봇 성공률 검증이 아니다. 시간 목표는 위 표처럼 조건별로 판정한다.
- **공동 시연 합의는 별도:** 팀원 1·3의 승인이나 직접 공동 참여를 대신 기록하지 않았다. 아래 명령은 재현용 후보이며 팀이 최종 채택했다고 표시하지 않는다.

### 최종 조건으로 실행·재검사

```bash
cd ~/vision-arm
source env.sh
colcon build --symlink-install
python scripts/run_team2_demo.py --gui --realistic --randomize --seed 5 --objects 9 --speed .6 --acceleration .7 --position-jitter-mm 4 --compact-return
```

GUI에서 **부품 9개 배치 → 검출 9개 확인 → 사이클 실행**. 종료는 실행 터미널 Ctrl+C다. Gazebo/RViz 실제 창은 `--gazebo-gui --rviz`를 추가한다. 창·다른 프로그램 부하가 달라지면 시간도 바뀐다.

자동 시험·통신 3회와 파일 저장까지 하려면 명령 끝에 다음을 추가한다.

```bash
--cycles 1 --transport-trials 3 --out logs/team2_recheck_seed5
```

다른 배치는 `--seed`를 바꾼다. 초기화는 물체를 교체하므로 같은 실행기에서 이동 중에는 하지 않는다. 별도 실행에서는 `--domain`·`--port`를 겹치지 않게 지정한다. 로그 재검사는 다음과 같다.

```bash
python scripts/review_team2_runtime.py logs/team2_recheck_seed5 --out logs/team2_recheck_seed5/runtime_review.json
```

전체 근거: `logs/team2_stability_20261123/summary.json`, seed3~5의 원본 결과·CSV·영상·라벨, `tests_final.xml`, `gui_confirmation/`. logs는 Git 제외 대상이다. 이번 추가 보완은 아직 커밋·푸시하지 않았다.

<a id="team2-20261123-shutdown-fix"></a>
## 2026-11-23 추가 — 재배치 응답 유실·종료 수명 보완

기준 HEAD는 `3cf84f4`다. 다른 담당의 `object_spawner.py`, `pick_place_node.py`, 비전 원본·GUI·기본 모델·보정·WEEK4 기록을 수정하지 않았다. 보호 대상 17개 파일을 HEAD와 바이트 비교했다. 변경은 팀원 2 어댑터·종료 관리자·전용 native 패키지와 실행 연결이다. 공유 sim/MoveIt 런치에는 선택 인자만 추가했으며 기존 기본 실행은 설치된 upstream 실행본을 계속 사용한다. 아래 팀원 2 실행 명령이 보완 경로를 선택한다.

### 1. 생성은 됐는데 응답을 놓친 재배치 오류

이전 두 번째 사이클은 `obj_7 스폰 요청 실패`로 중단됐다. Gazebo에는 `Created entity ... obj_7`과 `error sending response: Host unreachable`이 함께 기록됐다. 이름 중복이 원인이라고 확정하지 않는다.

새 `team2_spawner_adapter.py`는 원본 스폰 클래스를 상속하고 transport 요청 확인만 보완한다. 응답이 불확실하면 `/world/.../scene/info`에서 실제 이름·XY 위치(2 mm)·Z 범위·자세(5°)·primitive 형상 치수를 대조한다. 원하는 상태가 실제로 확인될 때만 성공으로 복구한다. 확인되지 않으면 실패를 유지한다. 삭제 응답 유실도 실제 모델이 사라졌을 때만 복구한다. 생성 명령을 즉시 재전송해 중복 모델을 만드는 방식은 사용하지 않는다. 확인 근거는 `TEAM2_GZ_REQUEST_RECOVERED`, 미확인은 `TEAM2_GZ_REQUEST_UNCONFIRMED`로 남긴다.

`--test-lost-create-reply obj_7:2`는 시험에서만 두 번째 obj_7 생성의 성공 응답을 일부러 버린다. 실제 서버 요청은 한 번만 보낸 뒤 장면을 확인한다. 정상 실행에는 이 옵션을 넣지 않는다. 실행기에서 상속된 시험용 환경변수도 제거해 우연한 유실 주입을 막았다.

### 2. MoveIt -11 — 코드와 객체 수명 순서

설치된 MoveIt 2.12.4의 종료 stack은 Executor 소멸자에서 SIGSEGV였다. upstream 종료 보고·공식 2.12.4 소스를 참고해 `src/team2_move_group` 전용 패키지를 추가했다. 원본 BSD 저작권·라이선스는 소스와 설치된 소스 사본에 보존했다.

- `team2_move_group.cpp`: capability가 만든 객체가 끝날 때까지 capability/controller 플러그인 loader를 유지한다. 외부 executor는 capability보다 먼저 정리한다.
- 계획 알고리즘·MoveIt 설정·컨트롤러 선택·ROS action/service 이름은 그대로 사용한다.
- `dlclose`를 막거나 SIGSEGV를 무시하거나 `_exit()`로 건너뛰는 수정이 아니다. 설치된 `/opt/ros/jazzy`도 덮어쓰지 않았다.

상세 근거·출처는 `src/team2_move_group/README.md`에 있다.

### 3. Gazebo·Python 종료 — 사용 중인 스레드 먼저 정리

`team2_gz_server.cpp`는 설치된 Gazebo Harmonic Server API를 사용하는 작은 수명 관리자다. UR 원본 런치의 URDF·스폰·컨트롤러 actions를 재사용하고 Gazebo 프로세스만 교체한다. 공식 ROS-native 서버 연결도 중간에 시험했으나 종료 문제가 남아, 최종본은 전용 lifecycle wrapper를 사용한다.

종료 시 ros2_control 통계 publisher 스레드를 ROS 시계가 살아 있을 때 join하고, ROS context를 닫아 controller executor를 깨운 다음 Gazebo 서버·플러그인을 정상 파괴한다. 통계 기능을 실행 중에 끄거나 오류 출력을 숨기지 않는다. 두 통계 registry는 지우지 않아 원본 controller manager가 정상 해제한다.

팀원 2 Python 노드는 wall-time 타이머로 Gazebo 시계 정지 중에도 신호를 처리한다. 노드를 executor에서 제거하고 worker pool을 먼저 join한 뒤 executor guard, MoveIt 클라이언트·TF, 노드, context 순서로 해제한다. 설치된 Jazzy executor가 guard를 먼저 파괴하는 순서 때문에 생길 수 있는 `Destroyable` 경쟁을 피한다. 외부 ROS context 종료는 팀원 2 TCP 브리지에서 예상한 종료 예외로 처리하며 다른 처리 예외는 숨기지 않는다.

`scripts/team2_shutdown.py`는 GUI·비전 → 이동 노드 → TCP/스폰 연결 → MoveIt → Gazebo → 브리지·남은 launch 순서로 정리한다. 통계 스레드가 끝날 때까지 clock bridge를 유지한다. 새 세션으로 만든 자기 프로세스 그룹만 대상으로 하며 다른 세션은 건드리지 않는 테스트도 추가했다. 강제 종료가 필요하면 결과에 남기며 성공으로 바꾸지 않는다.

스폰 원본의 TF 전용 스레드에서도 context 종료와 경합하는 예외가 관찰됐다. 팀원 2 어댑터 전용 프로세스에서 원본 main이 import하는 TF 팩토리를 spin_thread=False로 선택하고, TF의 Reentrant callback group과 원본 서비스를 하나의 4스레드 executor로 처리한다. 서비스 실행 중에도 TF를 갱신하며, 종료 시 worker → guard → TF → node/context 순서로 정리한다. 원본 파일은 바꾸지 않고 선택한 팩토리·spin 함수는 실패 시에도 복원한다. local import 연결을 잘못 짚은 중간 초기화 실패도 회귀 테스트로 보완했으며 로그를 보존했다.

실제 Ctrl+C에서는 자식 launch가 부모 TTY를 상속하면 이미 모든 자식이 SIGINT를 받았다고 판단해 신호 전달을 생략하는 사례도 확인했다. 자식 stdin을 DEVNULL로 분리해 Ctrl+C는 관리자만 받고, 자식에게 종료 신호를 한 번씩 보내도록 했다. GUI의 마우스·키보드 입력은 Qt 창을 통해 계속 받는다.

### 4. 반복 중 새로 발견한 그림자 오검출

재배치 응답 복구 후, 첫 사이클로 이동한 팔의 그림자가 너트 1개로 검출돼 9개 장면을 10개로 보는 문제가 추가로 발견됐다. 개수 검사가 집기를 막았으며 검사를 완화하지 않았다.

팀원 2 외형 개선용 비전 어댑터에서 연결 성분별 밝고 채도 낮은 금속 표면 지지를 확인한다. 어두운 그림자 성분만 제외한다. TF·스폰 정답·예상 위치로 검출을 고르거나 무조건 앞의 9개만 남기지 않는다. 실패 당시 실제 저장 영상은 기존 10개 → 보완 후 실제 부품 9개, 그림자 1개만 제외됨을 확인했다. 생성 금속 모델을 대상으로 한 보완이며 임의의 검은 실물 부품까지 검증한 필터는 아니다. 핵심 판정·수명 경계에 한국어 주석을 추가했다.

### 실행 방법

새 native 패키지가 있으므로 전체 빌드 후 환경을 다시 읽는다.

```bash
cd ~/vision-arm
source env.sh
colcon build --symlink-install
source env.sh
python scripts/run_team2_demo.py --gui --realistic --randomize --seed 5 --objects 9 --speed .6 --acceleration .7 --position-jitter-mm 4 --compact-return
```

GUI에서 부품 배치 → 검출 9개 확인 → 사이클 실행. 집기 완료·대기 자세 복귀 후 실행 터미널에서 Ctrl+C를 한 번 누른다. 종료 판정은 `summary.json`의 `shutdown_ok`, `shutdown.forced_groups`, `remaining_processes`, `runtime_log_review`에서 확인한다. 집기 성공·통신 KPI·시간 KPI·정상 종료는 서로 다른 판정이다.

응답 유실을 포함한 자동 반복 시험 명령은 아래와 같다. 이는 생성 성공 응답을 일부러 버리는 시험이며 정상 시연 옵션과 구분한다.

```bash
python scripts/run_team2_demo.py --gui --realistic --randomize --seed 5 --objects 9 --speed .6 --acceleration .7 --position-jitter-mm 4 --compact-return --cycles 2 --transport-trials 1 --test-lost-create-reply obj_7:2 --out logs/team2_shutdown_recheck
```

### 검증 범위·남은 제한

프로젝트 GUI는 offscreen 렌더링과 저장 화면으로 검사했다. Gazebo 서버·가상 카메라·실제 ROS/MoveIt 실행을 사용했으며 실물 로봇·실물 카메라 검증은 아니다. RViz·별도 Gazebo GUI 창을 함께 켠 조건의 종료까지 이번 결과로 확장하지 않는다.

중간 실패 로그는 삭제하지 않았다. 특히 개발 중 통신 100/100 전달에도 P95 1,226.63 ms·651.78 ms로 300 ms 목표를 넘은 측정이 있었다. 준비 이후 추가 관찰은 53.32 ms였고 다른 최종 측정도 별도로 남긴다. 최종 일부 통과가 모든 부하·시작 조건에서의 지연 보장은 아니다. Octomap 3D 센서 미설정과 일부 로봇 mesh 물리 충돌 형상 생성 제한은 이 종료 수정과 별개로 남아 있다. MoveIt 경로 충돌 검사는 유지한다.

### 2026-11-24 커밋 시점의 검증 상태

- Python 전체 회귀 테스트 **220개 통과**, native 포함 세 패키지 빌드와 신규 패키지 rosdep 확인 통과.
- 앞선 보완본의 9개 두 사이클은 18/18 집기 성공했고, 별도 3개 집기 후 실제 Ctrl+C 정상 종료도 확인했다. 그러나 후속 반복에서 스폰 TF 종료 경합과 초기화 입력 덮어쓰기 경쟁이 추가로 드러나 어댑터를 더 보완했다.
- 최신 변경은 `/team2/scene_request` → 같은 seq의 `/team2/scene_request_ack` → `/team2/spawn_exact` 전용 경로다. 실제 검출·TCP 목표 입력과 초기화를 분리하고, 기본 mutually-exclusive callback group에서 확인한 패킷을 고정해 원본 on_spawn을 호출한 뒤 이전 입력을 복원한다. 초기화 패킷과 빈 실제 검출이 뒤섞여 0개를 스폰하는 경쟁을 막기 위한 것이다. 원본 `/sim/spawn_objects` 서비스는 유지한다.
- **최신 전용 초기화 채널을 포함한 최종 end-to-end 반복 집기·Ctrl+C 재검증은 아직 완료하지 않았다.** 앞선 버전의 성공을 최신본 전체의 완료로 바꾸지 않는다. interactive 실행기의 `ok`는 실행 중에는 준비 상태이며, 별도 검사기의 집기 결과와 종료 후 `shutdown_ok`를 함께 확인해야 한다.
- 이번 커밋은 구현 및 현재 검증 범위를 저장하는 것이며, 종료·반복 안정성이 모든 조건에서 완전히 해결됐다는 선언이 아니다. `logs/`는 로컬 근거 자료로 Git에 포함되지 않는다.

### 2026-11-24 후속 로컬 검증: 센서 경고·전용 GUI 종료

- 팀원 2 native MoveIt 실행기에 `team2_known_geometry_only` 모드를 추가했다. MoveIt 2.12.4의 world geometry 초기화에서 Octomap만 생략하며 상태·부착 물체·등록 충돌 물체 감시는 유지한다. 깊이 센서로 미등록 장애물을 찾는 기능은 제공하지 않는다. 배포판 라이브러리와 다른 팀원 코드는 수정하지 않았다.
- `team2_gz_gui`는 Gazebo C++ GUI API를 직접 사용한다. 중간 seed 7 시험에서 `gz sim -g` 로더가 discovery/getenv 지점에서 시작 중 -11로 끝나 전용 실행기로 교체했다. 이 원인을 모든 Gazebo 오류의 원인으로 일반화하지 않는다.
- `team2_rviz`와 전용 launch는 화면·MoveIt 플러그인을 먼저 정리한 뒤 ROS를 종료한다. 원본 RViz 실행 파일과 공용 MoveIt launch는 그대로 둔다. 별도 `rviz.log`도 최종 오류 판정에 포함한다.
- 관리자 SIGTERM도 정리 경로를 타며, 종료 정리 중 추가 SIGINT/SIGTERM으로 정리가 끊기지 않게 했다. 강제 정리가 필요한 경우 실패 기록은 유지한다. 새 구독자 발견 지연으로 캡처가 실패한 사례는 대기 시간을 보완하고 누락 데이터 종류를 오류에 표시한다.
- 최종 `seed_8_fixed_lifecycle`: GUI·Gazebo·RViz, seed 8, 위치 ±4 mm, 9개 모두 성공·모델 ID 유지·최종 대기 자세 복귀. 센서 미설정 경고/충돌 재계획/비정상 종료/강제 종료/남은 프로세스 모두 0. **196.07초로 150초 목표는 미달**이다.
- 이후 `restart_signal_check`: 재실행 후 GUI·RViz 준비 확인 → SIGTERM → 추가 Ctrl+C 두 번. 정상 종료, 강제 종료와 남은 프로세스 0.
- 전체 Python 회귀 테스트 **222개 통과**, native 두 패키지 빌드 통과. 다른 세션을 종료하지 않는 검사와 별도 RViz 오류 수집 검사를 포함한다.
- 근거: `logs/team2_remaining_issues_recheck/final_review.json`과 각 실행 폴더. 중간 실패는 보존했다. seed 6/7은 부품 종류·각도·조명 변경이며 위치 격자는 같고, 실제 위치 변경 검증은 seed 8의 ±4 mm 시험이다. 모든 배치·부하에서 무충돌/정상 종료/150초 이내를 보장한 결과는 아니다. 종료 후 별도로 시도한 상태 유효성 조회는 서버 부재로 수행되지 않아 충돌 거부의 별도 음성 시험 결과로 사용하지 않는다.
- 이 항목은 후속 구현과 로컬 검증 결과의 기록이다. `logs/`의 실행 근거 자료는 Git에 포함하지 않는다.
