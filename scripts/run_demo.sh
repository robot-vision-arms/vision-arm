#!/bin/bash
# 시연 전체를 한 번에 켜고 끈다 (Gazebo·MoveIt·스폰·통신·검출·GUI). 어느 폴더에서 불러도 된다.
#   scripts/run_demo.sh            실물 폰 + YOLO (기본)          scripts/run_demo.sh rule   실물 폰 + 규칙
#   scripts/run_demo.sh sim        Gazebo 가상 카메라 (성민 님 run_team2_demo.py, 끌 때는 그 터미널에서 Ctrl+C)
#   scripts/run_demo.sh stop       끄기 (yolo·rule 로 켠 것)
# 바꿀 값 (앞에 붙인다):
#   ROTATE=270                     폰이 세로 영상을 보낼 때
#   CAMERA=rtsp://<폰 주소>:8554/live   params.yaml 의 camera_url 대신 (파일을 고치지 않는다)
#   SHEET=white                    흰 시트 (기본은 초록 시트)
#   CALIB_DIR=data/calib           보정 폴더 (intrinsics.npz, homography.npz)
#   IMAGE_DIR=<사진 폴더>           폰 대신 사진을 재생 (폰 없이 시험)
#   SIM_ARGS="gazebo_gui:=true"    Gazebo 창도 띄우기 (느려짐)
# 실물 모드는 검출 위치에 부품을 새로 만들어 집는다 (respawn). 집기 성공을 Pick 성공률로 쓰지 않는다.
# 나사 길이는 검출 상자로 재서 Gazebo 모델에 반영하고, Gazebo 에서 부품끼리는 서로 밀지 않는다 (성민 님 11-24).
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
MODE=${1:-yolo}
cd "$ROOT" || exit 1
mkdir -p logs
P='gui\.main_window|ros2 launch (ur5e|vision)_pick_place|vision_pick_place\.team2_sized_vision'
# 명령 앞부분만 맞춘다 (이 글자가 들어간 다른 셸·편집기를 시뮬레이터로 보지 않게)
SIM='^(/[^ ]*ruby .*|ruby .*)?(/[^ ]*/)?gz sim|^[^ ]*team2_gz_server'
Q="$P|$SIM|^[^ ]*move_group|object_spawner|team2_spawner_adapter|pick_place_node|robot_server|robot_bridge|robot_state_pub|parameter_bridge|static_transform"

find_pids() { pgrep -f "$1" | grep -vxE "$$|$PPID"; }   # 이 스크립트와 부른 셸은 뺀다

if [ "$MODE" = stop ]; then
  kill -INT $(find_pids "$P") 2>/dev/null
  for _ in $(seq 1 25); do find_pids "$SIM" >/dev/null || break; sleep 1; done
  kill -9 $(find_pids "$Q") 2>/dev/null
  echo "정리 완료"; exit 0
fi

source env.sh >/dev/null || exit 1
if [ "$MODE" = sim ]; then
  shift
  [ $# -gt 0 ] || set -- --realistic --randomize --seed 5 --objects 9 --speed .6 --acceleration .7 --position-jitter-mm 4 --compact-return
  exec python3 scripts/run_team2_demo.py --gui "$@"
fi
case "$MODE" in yolo|rule) ;; *) echo "쓰는 법: scripts/run_demo.sh [yolo|rule|sim|stop]"; exit 2;; esac
if find_pids "$SIM" >/dev/null; then echo "이미 시뮬레이터가 켜져 있습니다. 먼저 scripts/run_demo.sh stop"; exit 1; fi

CALIB_DIR=${CALIB_DIR:-data/calib}
for f in intrinsics.npz homography.npz; do
  [ -f "$CALIB_DIR/$f" ] || { echo "보정 파일이 없습니다: $CALIB_DIR/$f"; exit 1; }
done
C=src/vision_pick_place/config
PARAMS="--params-file $C/params.yaml"
[ "${SHEET:-green}" = white ] || PARAMS="$PARAMS --params-file $C/params_green.yaml"
if [ "$MODE" = yolo ]; then
  [ -f data/models/yolo_green.onnx ] && [ -f data/models/yolo_green.json ] || {
    echo "YOLO 모델이 없습니다: data/models/yolo_green.onnx · .json 을 받아 넣거나 scripts/run_demo.sh rule 로 켜세요."; exit 1; }
  PARAMS="$PARAMS --params-file $C/params_yolo.yaml"
fi

# 로그에 글자가 나올 때까지 기다린다. 오래 걸리면 로그를 보여 주고 끝낸다
wait_log() {   # 글자, 로그, 제한 시간(초)
  for _ in $(seq 1 "$3"); do grep -q "$1" "$2" 2>/dev/null && return 0; sleep 1; done
  echo "시작 실패 ($3초 동안 '$1' 없음). 로그: $2"; tail -5 "$2"; "$ROOT/scripts/run_demo.sh" stop; exit 1
}

PROFILE=$ROOT/logs/demo_profile
python3 scripts/make_team2_demo_profile.py --out "$PROFILE" --realistic > logs/demo_profile.log 2>&1 || { cat logs/demo_profile.log; exit 1; }
echo "Gazebo·MoveIt 켜는 중..."
nohup ros2 launch ur5e_pick_place sim.launch.py sim_params:=$PROFILE/sim.yaml world_file:=$PROFILE/workcell.sdf \
  description_file:=$ROOT/src/ur5e_pick_place/urdf/team2_ten_slots.urdf.xacro cycle_scene_mode:=respawn \
  ${SIM_ARGS:-gazebo_gui:=false} launch_rviz:=false > logs/demo_sim.log 2>&1 &
wait_log "충돌 물체" logs/demo_sim.log 90
echo "스폰·통신 켜는 중..."
nohup ros2 launch vision_pick_place team2_link.launch.py sim_params:=$PROFILE/sim.yaml asset_prefix:=$PROFILE/assets > logs/demo_link.log 2>&1 &
wait_log "스폰합니다" logs/demo_link.log 60
# 부품 슬롯 길들이기: 슬롯을 처음 쓸 때 부품이 패드에 붙었다 떨어진다. 로봇이 시작 자세일 때 한 번 쓰고 지운다 (WEEK4 동경 11-20)
timeout 60 ros2 service call /sim/warmup_slots std_srvs/srv/Trigger >/dev/null 2>&1
sleep 3
timeout 60 ros2 service call /sim/clear_objects std_srvs/srv/Trigger >/dev/null 2>&1
nohup python3 -m vision_pick_place.team2_sized_vision --ros-args -r __node:=vision_node $PARAMS \
  -p intrinsics_path:=$CALIB_DIR/intrinsics.npz -p homography_path:=$CALIB_DIR/homography.npz \
  ${IMAGE_DIR:+-p image_dir:=$IMAGE_DIR} ${CAMERA:+-p camera_url:=$CAMERA} ${ROTATE:+-p camera_rotate_deg:=$ROTATE} > logs/demo_vision.log 2>&1 &
if [ -z "${NO_GUI:-}" ]; then
  nohup python3 -m vision_pick_place.gui.main_window --source real --calib-dir $CALIB_DIR > logs/demo_gui.log 2>&1 &
fi
echo "준비 완료 ($MODE). 시트에 부품을 놓으면 Gazebo 에 나타납니다 → GUI [집기 시작]. 끄기: scripts/run_demo.sh stop"
