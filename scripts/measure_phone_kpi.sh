#!/bin/bash
# 필수 KPI: 실물 폰 입력 100회 송신 성공률과 검출→송신 P95 (기준 100 %, 300 ms). Gazebo 없이 잰다.
#   scripts/measure_phone_kpi.sh yolo        YOLO (시연과 같은 검출 노드)     scripts/measure_phone_kpi.sh rule   규칙
#   ROTATE=270 scripts/measure_phone_kpi.sh yolo     폰이 세로 영상을 보낼 때
# 바꿀 값: CAMERA=rtsp://<폰 주소>:8554/live (params.yaml 대신), COUNT=100, SHEET=white, CALIB_DIR=data/calib, IMAGE_DIR=<사진 폴더> (폰 없이 시험할 때. KPI 로 쓰지 않는다)
# 연결된 뒤 3초를 기다렸다가 센다 (시작 직후 제외, 결과에 settle_s 로 남음). 결과: logs/phone_kpi_<방식>_<시각>.json
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
MODE=${1:-yolo}; COUNT=${COUNT:-100}; CALIB_DIR=${CALIB_DIR:-data/calib}
cd "$ROOT" || exit 1
case "$MODE" in yolo|rule) ;; *) echo "쓰는 법: scripts/measure_phone_kpi.sh [yolo|rule]"; exit 2;; esac
source env.sh >/dev/null || exit 1
if pgrep -f '^(/[^ ]*ruby .*|ruby .*)?(/[^ ]*/)?gz sim|^[^ ]*team2_gz_server' >/dev/null; then
  echo "시뮬레이터가 켜져 있습니다. scripts/run_demo.sh stop 으로 끄고 재세요 (같이 켜면 지연이 커집니다)."; exit 1; fi
for f in intrinsics.npz homography.npz; do [ -f "$CALIB_DIR/$f" ] || { echo "보정 파일이 없습니다: $CALIB_DIR/$f"; exit 1; }; done
C=src/vision_pick_place/config
PARAMS="--params-file $C/params.yaml"
[ "${SHEET:-green}" = white ] || PARAMS="$PARAMS --params-file $C/params_green.yaml"
[ "$MODE" = yolo ] && PARAMS="$PARAMS --params-file $C/params_yolo.yaml"
export ROS_DOMAIN_ID=88                    # 다른 시연·팀원 노드와 섞이지 않게
STAMP=$(date +%Y%m%d_%H%M%S); OUT=logs/phone_kpi_${MODE}_$STAMP.json; LOG=logs/phone_kpi_${MODE}_$STAMP
mkdir -p logs
setsid ros2 launch vision_pick_place tcp.launch.py port:=19588 log_dir:=$LOG > $LOG.tcp.log 2>&1 & TCP=$!
setsid python3 -m vision_pick_place.team2_sized_vision --ros-args -r __node:=vision_node $PARAMS \
  -p intrinsics_path:=$CALIB_DIR/intrinsics.npz -p homography_path:=$CALIB_DIR/homography.npz \
  ${IMAGE_DIR:+-p image_dir:=$IMAGE_DIR} ${CAMERA:+-p camera_url:=$CAMERA} ${ROTATE:+-p camera_rotate_deg:=$ROTATE} > $LOG.vision.log 2>&1 & VIS=$!
finish() { kill -INT -- -$VIS -$TCP 2>/dev/null; sleep 2; kill -9 -- -$VIS -$TCP 2>/dev/null; }
trap finish EXIT
echo "재는 중 ($MODE, ${COUNT}회, 약 $((COUNT / 10 + 10))초)..."
python3 scripts/check_team_integration.py --count "$COUNT" --settle-s 3 --timeout $((COUNT / 3 + 60)) --out "$OUT" > $LOG.check.log 2>&1
python3 - "$OUT" "$MODE" "${IMAGE_DIR:-}" <<'PY'
import json, sys
path, mode, photos = sys.argv[1:4]
r = json.load(open(path))
if photos:
    print('[폰이 아니라 사진 재생입니다. KPI 로 쓰지 마세요]')
n, acked, want, p95 = (r.get(k) for k in ('observed_packets', 'acked_packets', 'required_packets', 'latency_p95_ms'))
if not n:
    print('검출 패킷이 하나도 오지 않았습니다. 폰 영상이 켜져 있는지, params.yaml 의 camera_url 이 맞는지 확인하세요.')
    sys.exit()
rate_ok = n == want and acked == want and r.get('communication_ok')
print(f'{mode}: 송신 {acked}/{want} ACK, 검출→송신 P95 ' + ('—' if p95 is None else f'{p95:.1f} ms') + f', 보정 {r.get("calib_ok_values")}')
print('  송신 성공률 100 %:', '달성' if rate_ok else '미달', '/ P95 300 ms 이하:',
      '—' if p95 is None else '달성' if p95 <= 300 and r.get('latency_ok') else '미달 (또는 지연 값이 덜 모임)')
if r.get('calib_ok_values') != [True]:
    print('  주의: 보정된 패킷이 아닙니다 (보정 파일·영상 크기 확인).')
for e in r.get('errors', [])[:5]:
    print('  오류:', e)
print('  결과:', path)
PY
tail -2 $LOG.vision.log | cut -c1-160
