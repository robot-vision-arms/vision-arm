# 2주차 구현·검증 기록

## 팀원 2 (성민) 공유 기록 — 2026-11-13

최신 공유 구현을 `~/vision-arm`에서 재현·검증했습니다. 아래 검출·분류 핵심 로직은 이번에 새로 작성한 것이 아니라 기존 공유 코드입니다.

| 항목 | 결과 |
|---|---|
| 빌드·테스트 재현 | 두 패키지 빌드 성공, 35개 테스트 통과 (실패·skipped 0) |
| 검출 설정 검증 | validation에서 threshold 60·80·100·120·Otsu 비교. 100의 F1 0.5083으로 기존 선택 재현 |
| 별도 test 평가 | 60장·정답 661개, 예측 979개·대응 419개. 정밀도 42.80%, 재현율 63.39%, 중심 MAE 14.62px, 장축 MAE 5.46° (314개) |
| 새로 추가한 도구 | scripts/preview_detections.py — 검출 박스·중심·장축을 표시하는 이미지 생성 |
| 시각 검토 | 접촉 물체가 합쳐지는 큰 박스, 작은 물체 누락, 배경 오검출 확인. 개선 완료로 판정하지 않음 |
| 추가 시뮬레이션 재현 | 기존 시험 스크립트의 지정 이동·흡착·해제 재시도 성공, 붙인 물체 약 0.09593m 상승 |
| 미완료 | 실제 촬영본의 나사·너트 분류 정확도, 보정 후 실측 mm·각도, 배경·접촉 물체 검출 개선 |

중심·각도 오차는 IoU로 대응된 물체 기준이며 실제 mm KPI나 실제 나사·너트 분류 정확도가 아닙니다.
상세 명령·증거 경로·제약은 [팀원 2 작업 기록](TEAM2_PROGRESS.md)에 기록했습니다.

2026-11-13 기준. 담당자 확정 여부와 별개로 PROJECT.md 8장의 2주차 기능을 진행했다.
실제 촬영 데이터와 시뮬레이션 검증을 구분해서 기록한다.

## 팀원 3 (가영) 공유 기록 — 2026-11-16

2주차 ROS2 발행(`vision_node.py`, `packet.py`) 담당 작업 결과입니다. 가영 PC 기준입니다.

| 항목 | 결과 |
|---|---|
| 빌드·테스트 | `0af3e57`에서 **55 tests** 통과. ChArUco 보정 PR #1 머지 후 62 tests, 작업 시트 ROI(`bf7c306`)·RViz 수정(`b0d63a7`) 머지 후 **68 tests, 0 errors, 0 failures, 0 skipped** (vision_pick_place 60, ur5e_pick_place 8) |
| 실제 폰 영상으로 `vision_node` 실행 | `params.yaml`(아이폰 RTSP `rtsp://172.20.10.1:8554/live`)로 실행. 시작 시 "미보정: px만 발행" 경고는 보정 파일이 없어서 정상 |
| `/vision/detections` | JSON 수신. `calib_ok: false`, 물체별 `part_type`·`u_px`·`v_px` 발행, `x_mm`·`y_mm`·`angle_deg`는 `null` |
| `/vision/debug_image` | 1280×720 영상에 검출 박스·라벨 표시 확인 |
| 발행 주기 | `ros2 topic hz`: 평균 9.998Hz (설정 10Hz). 측정 중 보인 약 5.3초 공백은 스트림 끊김이 아니라 WSL 시스템 시계 튐으로 인한 측정 착시로 판단 (아래 '시계 튐' 참고). 노드 오류 로그는 없음 |
| 작업 시트 ROI 영향 | `bf7c306`의 `sheet_roi`는 보정 파일이 있을 때만 적용된다. 미보정인 현재 폰 실행 결과(영상 전체 검출)에는 영향 없음 (코드 확인) |
| 제약 | 폰이 어두운 면을 비추고 있어 밝기 차이로 인한 오검출만 나왔다. 실제 부품 검출·분류 정확도, mm·각도는 미확인 |
| 대체 물체 시험 1차 (책상, 미보정) | 초콜릿(파란 포장, 둥근 모양) 검출·**nut**(장축/단축 1.01~1.05). 이클립스 케이스는 보라색 부분만 잡혀 **screw**(2.07). 흰 립밤은 밝은 책상과 구분 안 돼 미검출. 화면 가장자리 키보드·모니터 받침이 물체로 오검출. `gray`+Otsu는 밝은 배경 위 어두운 물체용이라 예상된 동작. 실제 부품 정확도가 아님 |
| 대체 물체 시험 2차 (책상, 미보정, 지연·시계 수정 후) | 립밤 대신 **치약(갈색 튜브)** 사용. 초콜릿 **nut**(1.03) ✓, 치약 전체 검출 **screw**(3.62) ✓, 이클립스는 1차와 달리 상자 전체가 잡혀 **screw**(1.71)지만 가운데 라벨 원이 별도 물체 **nut**(1.07)로 한 번 더 잡힘(상자 마스크 테두리가 끊겨 바깥 윤곽선이 분리된 것으로 보임). 화면 위 키보드, 아래 가방·의자, 왼쪽 지갑·소품이 물체로 오검출 — 보정 후 작업 시트 ROI(`bf7c306`)로 걸러질 대상. 노드 발행 1회에 물체 8개, 원본 재처리 11개(촬영 사이 폰이 조금 움직임). 실제 부품 정확도가 아님 |
| 영상 지연 원인 | 노드는 10Hz 타이머마다 `cap.read()` 1장만 꺼내는데 폰은 초당 약 27~28장을 보내, 읽지 못한 프레임이 버퍼에 쌓여 오래된 장면을 처리함 |
| 지연 수정 | `FrameSource`에 백그라운드 읽기 스레드 추가: 계속 읽어 **최신 프레임 1장만** 보관, `read()`는 새 프레임을 최대 `read_timeout_s`(기본 5초) 기다림, 끊기면 0.5초 뒤 재연결, `release()`가 스레드 종료. 이미지 폴더 입력은 그대로. 가짜 카메라 테스트 3개 추가(밀린 프레임 대신 최신 프레임, 끊김 후 재연결, 프레임 없을 때 시간 초과) → 전체 **71 tests, 0 failures** |
| 지연 수정 실측 (monotonic 시계) | 폰 송출 27.3fps, 20초 동안 멈춤 0회. 10Hz로 10초 읽은 뒤 수정 전은 버퍼에 8프레임(약 0.3초) 밀림, 수정 후는 항상 최신 프레임. `live_view.py`로 체감 지연도 괜찮음을 사용자 확인. 남은 지연은 주로 폰 앱 인코딩·전송 쪽(앱 비트레이트·키프레임 간격 설정으로 조정 가능, 해상도는 보정과 맞춰 1280×720 유지) |
| 시계 튐 원인·해결 | WSL 벽시계가 약 5.5초마다 +5.3초 → 0.3초 뒤 −5.8초로 튐. WSL의 `systemd-timesyncd`(NTP)와 Windows 호스트 시간이 약 5.5초 어긋나 서로 번갈아 시계를 고친 것. ROS 타이머는 시스템 시계를 따르므로 뒤로 튈 때마다 `vision_node`가 멈춤: 수정 전 `/vision/debug_image` 20초 동안 4.5Hz, 최대 간격 6.4초. `sudo systemctl disable --now systemd-timesyncd`로 WSL 쪽 동기화를 끔(Windows 시계만 따름) → 30초 동안 시계 튐 0회, `debug_image` **10.0Hz, 간격 평균 99ms·최대 115ms, 0.3초 넘는 끊김 0회**. 앞서 기록한 5.3초 공백도 같은 원인. 다른 PC는 확인 명령으로 각자 점검 필요(폰 연결과 무관하게 ROS 타이머·TCP ACK 대기·지연 KPI에 영향) |
| 스트림 끊김·재접속 실기 (지연 수정 후) | `vision_node` 실행 중 폰 앱 스트리밍을 껐다 켬. 90초 동안 `/vision/debug_image` 538장, 1초 넘는 공백 3회(2.0s·8.2s·12.0s). **세 번 모두 노드 재시작 없이 자동으로 수신 재개.** 끊긴 동안 노드는 `프레임을 읽지 못했습니다` 경고를 5초마다 1번 남기고 종료되지 않음. 측정 끝(75초)에 다시 끊긴 상태로 종료. 앱을 켠 시각을 따로 기록하지 않아 '켠 뒤 몇 초 만에 복구'는 측정 못 함(재연결 시도 간격은 코드상 약 0.5초+접속 시간). 개선 여지: 끊긴 동안 FFmpeg `Connection refused` 로그가 0.5초마다 찍혀 노드 로그가 지저분함 |
| 재접속 로그 정리 (11-17) | `FrameSource` 재연결 대기를 0.5초 고정 → **0.5초에서 시작해 실패마다 2배, 최대 5초**(`reconnect_delay_s`·`reconnect_max_delay_s`). 프레임을 받으면 0.5초로 되돌아감. 실제 FFmpeg로 닫힌 포트(`rtsp://127.0.0.1:8554/live`)에 20초 접속 시 `Connection refused` 로그 **40줄 → 7줄**. 대신 오래 끊겼다가 켜면 복구까지 최대 약 5초+접속 시간 걸림. 가짜 카메라 테스트 4개 추가(간격 증가·최대값·복구, 잘못된 설정 거부) → 전체 **106 tests, 0 failures**. 폰 실기 끊김·복구 재확인은 아직 안 함 |
| 남은 일 | 동경 보정 완료 후 mm·각도 발행 확인, (선택) 폰으로 재접속 실기 재확인 |

## 현재 상태

`colcon build --symlink-install`: 2개 패키지 성공.
`colcon test`와 `colcon test-result --verbose`: **47 tests, 0 errors, 0 failures, 0 skipped**.
색 기반 전처리·단계별 미리보기와 ROS2 검증을 추가한 최신 결과다. 초기 구현 검증은 35개였다.

| 작업 | 결과 |
|---|---|
| 중심·장축 검출, 면적 필터, 나사·너트 비율 분류 | 구현. 회전 물체·빈 영상·너트 구멍·퇴화 윤곽선 테스트 통과 |
| MVTec 정답 비교, Blur/Threshold 비교 | validation split에서 임계값 선택 후 test split 평가. 아래 수치 참고 |
| 보정 파일 로딩, 왜곡 보정 후 px→mm | 구현. mm 좌표에서 장축 각도 계산. 미보정 상태는 `calib_ok: false`, mm·각도는 `null` |
| 보정 파일 생성 스크립트 | 구현. 합성 체커보드 16장으로 실행 확인(RMS 0.4244px, ChArUco 전환 전), 합성 기준점으로 호모그래피 CLI 검증. 현재 기본은 ChArUco 보드·ArUco 작업 시트. 실제 폰 보정은 미실시 |
| ROS2 `/vision/detections`, `/vision/debug_image` | 구현. 실제 ROS 퍼블리셔·구독자로 보정 유무 두 경우 수신 확인 |
| 작업면→base_link, 툴 자세 | 구현. 단위·축 회전·쿼터니언 테스트 통과 |
| 지정 좌표 이동 `/sim/move_to_target` | Gazebo + MoveIt에서 실제 실행 성공. 패드 끝 높이 0.25m → 0.35m |
| 확인 ② 늦게 스폰한 부품 attach/detach | 통과. attached→detached→attached→detached 상태 확인. 붙은 부품 약 0.0992m 상승 후 분리·낙하 확인 |
| 자체 부품 촬영·실제 mm/각도 확인 | 남음. 폰 영상 주소·고정 거치·ChArUco 보드 및 작업 시트 촬영 필요 |
| 부품 재생성 후 재흡착, 자동 스폰·접근·분류 사이클 | 미검증/후속 구현. 3~4주차 범위 |

미보정 상태에서도 픽셀 검출과 디버그 영상은 확인할 수 있다. 보정 파일은 시작할 때 읽으므로 새 보정값을 만들면 노드를 재시작한다.
합성 데이터용 보정값은 `/tmp`에만 생성했고 실제 작업용 `data/calib`에는 넣지 않았다.
새 보정 스크립트의 해상도 기록과 실제 입력 해상도가 다르면 프레임 처리를 거부한다.

## 실행 순서

저장소 루트(`~/vision-arm`, 다른 곳에 clone 했다면 [WEEK1.md](WEEK1.md) 의 링크 방법 참고)에서 실행한다.

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

이 설정은 MVTec용이며 `calib_ok: false`가 정상이다. 최신 설정은 `metal_hsv` 모드다.
기본 폰 카메라 설정은 `gray` 모드·Otsu이며 형태 연산도 꺼져 있다.
GUI·TCP 브리지는 후속 주차의 뼈대 상태이므로 이 확인에는 비전 노드를 단독 실행한다.

실제 폰 영상과 mm 좌표 확인:

1. `PYTHONPATH=src/vision_pick_place python3 scripts/make_calib_sheets.py` 로 `data/calib/charuco_board_A4.pdf`, `data/calib/work_sheet_A4.pdf` 를 만들어 A4 가로 100% 로 인쇄하고, 100 mm 선을 자로 확인한다. ChArUco 보드는 단단한 판에 평평하게 붙인다.
2. 같은 해상도·줌으로 ChArUco 보드를 손에 들고 15~20장 찍어 `data/calib/charuco/` 에 넣고 `python scripts/calibrate_camera.py --images data/calib/charuco` → `intrinsics.npz` (카메라 내부 파라미터·왜곡).
   폰을 최종 위치에 고정하고 작업대에 붙인 작업 시트를 부품 없이 1장 찍어 `python scripts/compute_homography.py --aruco --image data/calib/sheet.png` → `homography.npz` (2D 호모그래피).
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

## MVTec 검출 평가 (초기 밝기 기준)

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

## 색 기반 전처리 개선 (2026-11-13 후속 작업)

현재 로컬과 원격 `main`이 일치함을 `git fetch origin` 후 확인하고 진행했다.
원본 영상의 밝기만으로는 나무 무늬·그림자가 부품과 섞여 HSV의 채도와 명도를 함께 사용했다.
`metal_hsv`는 **채도 ≤ 70 또는 명도 ≤ 90**인 픽셀을 선택하고, 3×3 close → 3×3 open을 적용한다.
Blur 커널은 5, 최소 면적은 500px²다. 이 모드는 `threshold` 값을 사용하지 않는다.

validation 55장(정답 647개)에서 12개 후보를 F1으로 비교했다.
기존 gray와 close/open 조합 4개, HSV 채도·명도 조합 8개 중 선택한 설정의
정밀도는 76.75%, 재현율은 79.60%, F1은 78.15%였다.
최종 선택 후 같은 test 60장(정답 661개)을 평가했다. 초기 결과를 이미 확인한 데이터이므로
아래 수치는 새로 수집한 독립 테스트셋의 성능을 의미하지 않는다. test 결과로 설정을 고르지는 않았다.

| test 60장, 회전 IoU ≥ 0.5 | 기존 gray·임계값 100 | 새 metal_hsv |
|---|---:|---:|
| 검출 수 | 979 | 710 |
| 정답 대응 수 | 419 | 551 |
| 정밀도 | 42.80% | 77.61% |
| 재현율 | 63.39% | 83.36% |
| F1 | 51.10% | 80.38% |
| 대응 물체 중심 오차 | 14.62px | 9.50px |
| 대응 물체 장축 각도 오차 | 5.46° (314개) | 5.20° (322개) |

선택된 설정은 `params_mvtec.yaml`에 반영했고 같은 파라미터를 ROS2 노드·평가·미리보기 도구에 연결했다.
실제 ROS 송수신 테스트에서도 색 배경 위의 금속 부품이 검출·발행되는지 확인했다.
폰 카메라의 기본 `gray` 설정은 유지했다. 흰 배경은 저채도이므로 `metal_hsv`에서 배경 전체가
전경으로 선택될 수 있다. 금색 부품의 반사·그림자, 겹친 물체의 분리도 여전히 한계다.
이 결과는 검출 평가이며 실제 나사·너트 분류 정확도나 mm KPI가 아니다.

재현 명령 (저장소 루트):

```bash
# validation에서만 설정 선택. 결과에는 12개 후보와 선택된 설정을 모두 저장한다.
PYTHONPATH=src/vision_pick_place:$PYTHONPATH python scripts/tune_detector.py \
  --validation data/mvtec_screws/mvtec_screws_val.json --images data/mvtec_screws/images

# 선택한 설정으로 test 평가
PYTHONPATH=src/vision_pick_place:$PYTHONPATH python scripts/evaluate_detector.py \
  --annotations data/mvtec_screws/mvtec_screws_test.json --images data/mvtec_screws/images \
  --segmentation-mode metal_hsv --hsv-max-saturation 70 --hsv-max-dark-value 90 \
  --close-ksize 3 --open-ksize 3 --out logs/week2_detector_test_hsv.json

# 원본과 이진화 영상 비교
python scripts/preview_camera.py --count 2 --segmentation-mode metal_hsv \
  --hsv-max-saturation 70 --hsv-max-dark-value 90 --close-ksize 3 --open-ksize 3 \
  --out data/preview_hsv
```

선택 기록은 `logs/week2_preprocessing_validation.json`, 결과 이미지는 `data/preview_hsv`에 저장한다.
다음 실측 단계는 여전히 **폰 고정·ChArUco 내부 파라미터 캘리브레이션·ArUco 작업 시트 호모그래피 → 자체 나사·너트 촬영 → 좌표·각도·종류 검증**이다.

## 단계별로 눈으로 확인하기

ROS나 카메라 없이 실행하는 학습용 화면을 추가했다. 저장소 루트에서:

```bash
source env.sh
python scripts/preview_steps.py
```

기본 입력은 보유 MVTec 사진이며 `이전 사진`·`다음 사진`으로 넘길 수 있다.
`내 사진 열기`로 자체 촬영 사진도 선택할 수 있다. 흰 배경의 자체 사진은 `gray` 방식으로 시작한다.

1. 왼쪽 목록에서 원본 → Blur → S·V 채널 → 이진화 → CLOSE → OPEN → 검출을 하나씩 선택한다.
   S와 V는 Blur에서 각각 분리한 별도 채널이며 서로를 차례로 변환한 것이 아니다.
2. 왼쪽 영상은 선택 단계의 입력, 오른쪽은 출력이다. 비교 기준을 원본 영상으로 바꿀 수도 있다.
3. 채도·밝기·최소 면적 값을 바꾸면 즉시 다시 계산한다. Blur=1, CLOSE/OPEN=0으로 연산을 끄고 차이를 본다.
4. 아래 코드·설명을 실제 화면과 함께 확인한다. 영상 위 휠로 확대하고 드래그로 이동한다. 더블클릭하면 전체 보기로 돌아간다.
5. `현재 설정의 모든 단계 저장`을 누르면 `data/pipeline_steps/<사진 이름>/`에 PNG들과 `steps.txt`를 저장한다.

화면은 `camera_stream.preprocess()`의 `on_step` 콜백으로 실제 중간 결과를 받아 사용한다.
전처리 알고리즘을 GUI에 별도로 복사하지 않았다. 단계 출력과 일반 전처리의 최종 마스크가 동일한지 테스트한다.
검출 화면은 동일한 `find_objects`·`classify_part` 함수를 사용한다. 실제 mm 보정 평가는 이 도구의 범위가 아니다.
GUI는 창 없는 환경에서도 파라미터 변경·사진/방식/단계 전환·확대 유지·저장을 시험했다.
한글 폰트가 없는 WSL에서는 설치된 Windows 맑은 고딕을 읽어 사용할 수 있다. 폰트 파일은 저장소에 복사하지 않는다.

창 없이 단계 이미지들만 저장하거나 특정 사진을 열려면:

```bash
python scripts/preview_steps.py --export-only
python scripts/preview_steps.py --image data/captures/part.png --mode gray
```

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

## 팀원 1 (동경) 다음 작업 계획 — 2026-11-16 작성

2주차 담당(`grasp.py`, `/sim/move_to_target`)은 완료. 아래는 1주차에서 넘어온 일과 3주차(2026-11-23 ~ 11-29) 담당 작업이다.

### 1주차에서 넘어온 일 (3주차 전에 끝낼 것)

| 작업 | 내용 | 완료 기준 |
|---|---|---|
| 실제 캘리브레이션 | 11-17 촬영 예정. ChArUco 보드 15~20장 → `calibrate_camera.py`, 폰 고정 후 작업 시트 1장 → `compute_homography.py --aruco --debug-out` ([PR #1](https://github.com/robot-vision-arms/vision-arm/pull/1)) | `data/calib/intrinsics.npz`, `homography.npz` 생성. RMS(px), 시트 RMS·한 점씩 뺀 오차(mm)를 WEEK1에 기록 |
| 작업 시트 마커 마스크 / 검출 ROI | 작업 시트 귀퉁이 ArUco 마커(검은 사각형)가 `vision_detector` 에 부품으로 잡히지 않게 한다. 호모그래피 역변환으로 마커 영역(mm 위치는 `aruco_calib.SHEET_MARKERS`)을 이미지에서 가리거나, 검출 ROI 를 시트 안쪽으로 제한 | 시트만 찍은 사진에서 검출 0개, 시트 위 부품 사진에서 마커 오검출 0개. 테스트 추가. 검출 로직(성민)과 인터페이스만 맞춘다 |
|  ↳ 11-16 진행 | 구현: `workspace_roi.py` (시트 안쪽 − 마커 영역 마스크), `Calibrator.mm_to_px`, `process_frame(roi_mask=...)`, `vision_node` 파라미터 `sheet_roi`·`sheet_edge_margin_mm`(3)·`sheet_marker_margin_mm`(5). 디버그 영상에 노란 경계선 | 합성 사진 테스트 6개 통과: ROI 없이는 마커 4개·시트 밖 물체 오검출 → ROI 적용 시 0개, 시트 위 나사·너트는 mm 오차 1 mm 미만·각도 1.5° 미만. **실제 촬영으로 여백 조정 남음** |

### 2주차 보강: 지정 좌표 이동 재시도 (11-16)

시뮬레이터를 막 띄운 직후 `/sim/move_to_target` 을 부르면 OMPL 이 가끔 `Unable to solve the planning problem` 으로 실패했다 (11-16 띄운 직후 3~16초에 부른 7번 중 2번 실패).

| 바꾼 것 | 내용 |
|---|---|
| 계획 실패 시 다시 시도 | `plan_attempts`(3), `plan_retry_delay_s`(2초). MoveIt 이 IDLE(실행 중 아님)이고 실패 코드가 계획 실패일 때만. Jazzy OMPL 은 계획 실패를 `PLANNING_FAILED` 가 아니라 범용 `FAILURE`(99999) 로 돌려줘서 이것도 포함 (`move_group` 로그로 확인). `CONTROL_FAILED`·`NO_IK_SOLUTION` 은 다시 해도 같으므로 제외 |
| 관절 상태 대기 | 첫 이동 전에 `/joint_states` 를 최대 `joint_state_wait_s`(10초) 기다림 |
| 계획 시간 파라미터 | 고정 5초 → `planning_time_s`(5초) |
| 실패 메시지 | `error_code` 와 pymoveit2 실패 이유를 같이 표시 |

확인: `planning_time_s` 를 0.0001초로 낮춰 매번 실패시키면 코드 99999 → 2/3, 3/3 재시도 → 실패 보고, 5초로 되돌리면 노드 재시작 없이 바로 성공. 서비스가 뜬 뒤 10초에 `check_sim_week2.py` 전체 통과. colcon 68개 통과.
자연 발생 실패도 재시도로 풀리는 것을 확인했다 (아래 충돌 물체 확인 중 첫 호출: 99999 실패 → 2초 뒤 2/3 에서 성공).
같은 확인 중 `INVALID_MOTION_PLAN`(-2) 도 나왔다. 찾은 경로를 MoveIt 이 다시 검사했더니 중간에 `forearm_link` 와 `vacuum_pad` 가 닿아서 실행하지 않고 버린 경우로, 로봇은 안 움직였으므로 재시도 대상에 추가했다.

### 2주차 보강: 상자·바닥을 MoveIt 충돌 물체로 등록 (11-16)

MoveIt 은 Gazebo world 파일을 읽지 않아서, 그대로 두면 상자 벽을 뚫고 가는 경로를 계획할 수 있다 (RViz 에도 상자가 안 보였다).
`ur5e_pick_place/workcell_scene.py` 가 `worlds/workcell.sdf` 의 static 모델 충돌(box·plane) 을 읽고, `pick_place_node` 가 시작할 때 move_group 이 `/collision_object` 를 구독하면 등록한다.
크기·위치를 코드에 따로 적지 않으므로 **sdf 에서 상자를 옮기거나 늘리면 MoveIt 에도 자동 반영**된다. 바닥 plane 은 윗면이 z=0 보다 2 mm 아래인 2 cm 판으로 바꾼다 (z=0 에 맞추면 로봇 베이스가 시작부터 충돌로 잡힘). `work_mat` 은 충돌 없는 표시용이라 빠진다.

| 확인 (나사 상자, base (0, 0.45)) | 결과 |
|---|---|
| `/get_planning_scene` | 충돌 물체 11개 (상자 2 × 바닥·벽 4 + 바닥 판) |
| 상자 위 15 cm / 상자 안쪽 가운데 바닥 위 2 cm | 둘 다 성공 (시도 1회) |
| 패드 끝을 벽(`wall_px`) 속 1 cm 높이에 | 거부: `Unable to sample any valid states for goal tree` (목표 자세가 충돌) |

colcon 72개 통과 (`test_workcell_scene.py` 4개 추가).

### 3주차: GUI (`gui/main_window.py`, `gui/ros_worker.py`)

현재 두 파일은 뼈대(TODO)만 있다. 과제 필수 4화면 (PROJECT.md 2.2):

| 화면 | 데이터 출처 | 표시 내용 |
|---|---|---|
| ① 영상 | `/vision/debug_image` (Image) | 검출 ROI·중심·장축 화살표가 그려진 영상 |
| ② 좌표 표 | `/vision/detections` (String·JSON, `packet.py`) | id, part_type, u/v px, x/y mm, angle_deg. `calib_ok: false` 면 mm·각도 칸을 비우고 "미보정" 표시 |
| ③ 송수신 기록 | `/robot/log` `{"seq","sent","acked","latency_ms","error"}`, `/sim/pick_log` `{"seq","obj","part_type","result","plan_ms","error"}` | 최근 N건, 실패는 색으로 구분, 지연 ms |
| ④ 캘리브 상태 | 패킷 `calib_ok` + 보정 파일 | 내부 파라미터·호모그래피 로드 여부, RMS(px), 재투영 오차(mm), 보정 파일 생성 시각 (카메라가 움직였는지 판단용, PROJECT.md 11장) |

구현 순서:

1. `ros_worker.py`: QThread 안에서 `rclpy` 노드 생성 → 네 토픽 구독 → 콜백에서 Signal emit → `spin_once` 루프. `stop()` 은 플래그 → `rclpy.try_shutdown()` → `wait()`
2. `main_window.py`: 위 4화면 배치, Signal → 위젯 갱신 슬롯 연결, 창 닫을 때 워커 정지
3. 이미지 변환(BGR numpy → QImage)은 `gui/pipeline_inspector.py` 에 있는 방식과 맞춘다
4. 테스트: 워커 없이 슬롯에 가짜 패킷·로그를 넣어 표·상태 갱신 확인 (`QT_QPA_PLATFORM=offscreen`), 실제 ROS 로 `vision_node` 발행 → GUI 수신 확인
5. 문서: 실행 명령, 화면 캡처를 WEEK3 기록에 남김

의존성: ③의 `/robot/log` 는 성민 님 3주차 `robot_bridge.py` 가 발행한다. 먼저 나오기 전까지는 가짜 발행(`ros2 topic pub`)으로 확인한다.

완료 기준 (PROJECT.md 8장 3주차): GUI 에 영상·좌표·캘리브 상태가 실시간으로 갱신되고, 송수신 기록이 쌓인다.

### 4주차 (참고)

KPI 측정(`evaluate_kpi.py`), 100회 송신 테스트. 3주차 GUI 와 `robot_bridge` CSV 가 입력이다.
