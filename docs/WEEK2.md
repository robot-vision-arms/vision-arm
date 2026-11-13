# 2주차 구현·검증 기록

2026-11-13 기준. 담당자 확정 여부와 별개로 PROJECT.md 8장의 2주차 기능을 진행했다.
실제 촬영 데이터와 시뮬레이션 검증을 구분해서 기록한다.

## 현재 상태

`colcon build --symlink-install`: 2개 패키지 성공.
`colcon test`와 `colcon test-result --verbose`: **35 tests, 0 errors, 0 failures, 0 skipped**.

| 작업 | 결과 |
|---|---|
| 중심·장축 검출, 면적 필터, 나사·너트 비율 분류 | 구현. 회전 물체·빈 영상·너트 구멍·퇴화 윤곽선 테스트 통과 |
| MVTec 정답 비교, Blur/Threshold 비교 | validation split에서 임계값 선택 후 test split 평가. 아래 수치 참고 |
| 보정 파일 로딩, 왜곡 보정 후 px→mm | 구현. mm 좌표에서 장축 각도 계산. 미보정 상태는 `calib_ok: false`, mm·각도는 `null` |
| 보정 파일 생성 스크립트 | 구현. 합성 체커보드 16장으로 실행 확인(RMS 0.4244px), 합성 기준점으로 호모그래피 CLI 검증. 실제 폰 보정은 미실시 |
| ROS2 `/vision/detections`, `/vision/debug_image` | 구현. 실제 ROS 퍼블리셔·구독자로 보정 유무 두 경우 수신 확인 |
| 작업면→base_link, 툴 자세 | 구현. 단위·축 회전·쿼터니언 테스트 통과 |
| 지정 좌표 이동 `/sim/move_to_target` | Gazebo + MoveIt에서 실제 실행 성공. 패드 끝 높이 0.25m → 0.35m |
| 확인 ② 늦게 스폰한 부품 attach/detach | 통과. attached→detached→attached→detached 상태 확인. 붙은 부품 약 0.0992m 상승 후 분리·낙하 확인 |
| 자체 부품 촬영·실제 mm/각도 확인 | 남음. 폰 영상 주소·고정 거치·체커보드 및 격자 촬영 필요 |
| 부품 재생성 후 재흡착, 자동 스폰·접근·분류 사이클 | 미검증/후속 구현. 3~4주차 범위 |

미보정 상태에서도 픽셀 검출과 디버그 영상은 확인할 수 있다. 보정 파일은 시작할 때 읽으므로 새 보정값을 만들면 노드를 재시작한다.
합성 데이터용 보정값은 `/tmp`에만 생성했고 실제 작업용 `data/calib`에는 넣지 않았다.
새 보정 스크립트의 해상도 기록과 실제 입력 해상도가 다르면 프레임 처리를 거부한다.

## 실행 순서

저장소 루트에서 실행한다. 이 PC의 실제 경로는 `/home/user1/mini_projects3/vision-arm`이다.

```bash
source env.sh
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

카메라 없이 MVTec 입력으로 비전 노드만 실행:

```bash
ros2 run vision_pick_place vision_node --ros-args --params-file src/vision_pick_place/config/params_mvtec.yaml
```

별도 터미널에서 같은 환경을 source한 뒤:

```bash
ros2 topic echo /vision/detections --once
ros2 topic info /vision/debug_image
```

이 설정은 MVTec용이며 `calib_ok: false`가 정상이다. 기본 폰 카메라 설정의 Otsu 값은 바꾸지 않았다.
GUI·TCP 브리지는 후속 주차의 뼈대 상태이므로 이 확인에는 비전 노드를 단독 실행한다.

실제 폰 영상과 mm 좌표 확인:

1. 폰을 최종 위치에 고정하고 같은 해상도로 체커보드 15~20장, 격자 사진과 기준점 CSV를 준비한다.
2. `scripts/calibrate_camera.py`, `scripts/compute_homography.py`의 사용 예대로 보정 파일을 생성한다.
3. `src/vision_pick_place/config/params.yaml`의 `camera_url`과 보정 경로를 설정한다.
4. `ros2 run vision_pick_place vision_node --ros-args --params-file src/vision_pick_place/config/params.yaml` 실행.
5. 토픽의 `calib_ok: true`와 실측 mm·각도를 비교한다. 보정점 재투영 오차만으로 위치 KPI 통과를 선언하지 않는다.

시뮬레이터의 이동·흡착 시험:

```bash
ros2 launch ur5e_pick_place sim.launch.py gazebo_gui:=false launch_rviz:=false
```

별도 터미널에서 환경을 source한 뒤:

```bash
# 기본 지정 좌표 (100, 100)mm, 패드 끝 높이 0.25m
ros2 service call /sim/move_to_target std_srvs/srv/Trigger

# obj_0이 없는 새 시뮬레이터에서만 실행: 스폰·흡착·이동·낙하까지 확인
python scripts/check_sim_week2.py
```

확인 스크립트는 목표 높이를 0.25m에서 0.35m로 바꾸고 부품 `obj_0`을 남긴다. 재실행할 때는 시뮬레이터를 재시작한다.
결과는 `logs/week2_sim_check.json`에 저장한다. 실제 실행한 높이와 tool0 목표도 콘솔에 출력한다.
흡착 시험은 고정 관절의 연결 메커니즘 확인이다. 바닥으로 떨어진 부품을 다시 연결해 올리므로
패드가 부품과 접촉하는 집기 위치·접근 궤적의 정확도를 검증한 것은 아니다.
기존 `/sim/run_cycle`의 전체 집기·분류 사이클은 아직 구현 전이며 이 서비스는 그 점을 응답한다.

## MVTec 검출 평가

데이터에 동봉된 README의 회전 박스 `(row, col, width, height, phi)` 정의를 사용했다.
픽셀 좌표 원점의 0.5px 차이와 phi의 부호를 반영하고, 회전 IoU ≥ 0.5인 박스를 IoU 순으로 일대일 대응했다.
중심 오차는 대응한 박스의 중심 거리 평균, 각도 오차는 대응한 정답 중 장단축 비율 > 1.5인 물체만 계산했다.

validation split에서 threshold 60·80·100·120·Otsu를 비교해 F1이 가장 높은 **100**을 선택했다.
Blur 커널 5, 최소 면적 500px²는 고정했다. test split은 선택에 사용하지 않았다.

| test split 60장, 정답 661개 | Otsu | 고정 임계값 100 |
|---|---:|---:|
| 검출 수 | 3767 | 979 |
| 대응 성공 수 | 201 | 419 |
| 정밀도 | 5.34% | 42.80% |
| 재현율 | 30.41% | 63.39% |
| 대응 물체 중심 오차 | 18.33px | 14.62px |
| 대응 물체 장축 각도 오차 | 6.78° (128개) | 5.46° (314개) |

나무 무늬·그림자와 붙어 있는 부품이 단순 이진화의 한계다. 이 수치는 실측 mm KPI나 나사·너트 분류 정확도가 아니다.
MVTec의 13개 클래스는 이 프로젝트의 두 분류명과 직접 대응시키지 않았다. 실제 나사·너트 사진에서 비율 기준을 따로 확인해야 한다.

재현 명령:

```bash
PYTHONPATH=src/vision_pick_place:$PYTHONPATH python scripts/evaluate_detector.py \
  --annotations data/mvtec_screws/mvtec_screws_test.json \
  --images data/mvtec_screws/images --threshold 100 --out logs/week2_detector_test_tuned.json
```

`logs/week2_detector_test.json`은 Otsu 결과, `logs/week2_detector_validation.json`은 이번 validation 비교 기록이다.
logs와 원본 데이터는 git에 포함하지 않는다.

## 시뮬레이터에서 수정한 문제

- 흡착 패드가 추가된 URDF와 기본 SRDF의 로봇 이름을 `ur5e`로 일치시켰다.
- 패드와 고정 장착 부위(`wrist_3_link`, `flange`, `tool0`)의 충돌만 SRDF에서 제외했다. 다른 링크의 충돌 검사는 유지한다.
- 로컬 `moveit.launch.py`가 패드를 포함한 URDF·SRDF를 명시적으로 전달한다.
- upstream control 런치가 URDF를 YAML로 해석할 때 오류를 일으키던 XML 주석의 `: `를 제거했다.
- 기본 계획 시간 0.5초를 5초로 늘렸다. 이 PC에서 낮은 목표 높이 0.10m는 계획에 실패했고,
  2주차 이동 확인의 기본 높이는 실제 성공한 0.25m로 정했다. 부품 바로 위까지의 하강 경로는 추가 검증이 필요하다.
- 부품이 스폰되기 전 경고 반복을 끄고 슬롯별 상태 토픽을 추가했다.
  설정 근거는 [Gazebo Harmonic DetachableJoint 공식 API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DetachableJoint.html)다.

Gazebo는 headless로 검증했다. GUI 화면·RViz 조작은 직접 확인하지 않았다.
현재 Gazebo DART의 로봇 STL 충돌 메시 생성 경고와 MoveIt의 3D 센서 미설정 Octomap 오류가 남아 있다.
이번 이동과 관절 기반 흡착 시험은 성공했지만, Gazebo에서 팔 메시의 물리 충돌과 작업 셀 벽을 포함한 계획 장면은 추가 검증 대상이다.
