# 팀원 2 (성민) — 1·2주차 실행 결과

확인일: 2026-11-13. 실행 위치: `/home/user1/vision-arm`.
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
따라서 이동의 반복 성공률·충돌 안전성까지 입증한 것은 아니다. STL 충돌 메시 경고와 센서 미설정 Octomap 오류도 추가 검증 대상이다.

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
- [ ] 이동 반복 성공률, 작업 셀 충돌 장면·물리 충돌 검증.

코드·자동 재현 시험은 진행했지만 실제 촬영 자료와 화면 확인이 필요한 항목을 전체 완료로 판정하지 않는다.
