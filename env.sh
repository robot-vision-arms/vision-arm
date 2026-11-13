# 작업할 때마다 터미널에서:  source ~/vision-arm/env.sh   (저장소를 다른 곳에 뒀다면 ~/vision-arm 링크를 걸거나 그 경로로)
# 순서가 중요하다: conda 해제 → ROS2 → venv → 워크스페이스 빌드 결과
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"   # -P: 링크로 들어와도 실제 경로를 써서 colcon 빌드 경로가 한 가지로 유지됨

# conda 의 python 이 PATH 앞에 있으면 rclpy(시스템 Python 3.12 용)를 못 불러온다
while [ -n "${CONDA_PREFIX:-}" ]; do conda deactivate; done

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/.venv/bin/activate"
[ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"

# WSL2 에서 Gazebo 화면이 깨지거나 검게 나오면 아래 줄 주석을 풀어 소프트웨어 렌더링으로 띄운다 (느려짐, docs/SETUP.md Step 5)
# export LIBGL_ALWAYS_SOFTWARE=1

cd "$WS_DIR"
echo "[env] python=$(which python3)  ROS_DISTRO=${ROS_DISTRO}  ws=$WS_DIR"
