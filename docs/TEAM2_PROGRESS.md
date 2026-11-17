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
