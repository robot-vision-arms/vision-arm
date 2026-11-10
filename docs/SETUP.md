# 개발 환경 구축 가이드 (WSL2 Ubuntu 24.04 + ROS2 Jazzy + PySide6 + Gazebo Harmonic + MoveIt2)

팀원 3명이 같은 순서로 따라 하면 같은 환경이 되도록 정리한 문서입니다. 순서만 빠르게 따라가려면 [WEEK1.md](WEEK1.md) 체크리스트를 보세요.
처음 받은 가이드에서 **바꾼 부분은 맨 아래 [원본 가이드에서 바꾼 점](#원본-가이드에서-바꾼-점)** 에 이유와 함께 적었습니다.

## 환경 사양

| 항목 | 값 |
|---|---|
| OS | Windows 11 + WSL2, Ubuntu 24.04 LTS (Noble) |
| ROS | ROS2 Jazzy Jalisco (Desktop) |
| Python | **시스템 Python 3.12** + 프로젝트 venv (`.venv`). conda 는 이 프로젝트에서 쓰지 않음 |
| 주요 라이브러리 | OpenCV·NumPy (apt, ROS 와 같은 빌드), PySide6 (pip, venv) |
| 시뮬레이터 (확장) | Gazebo Harmonic (Jazzy 공식 짝) + `ur_simulation_gz` (UR5e) + MoveIt2 + pymoveit2, 전부 apt (rosdep) |
| 권장 사양 | WSL 메모리 8 GB 이상, 디스크 여유 15 GB 이상 (ROS Desktop + Gazebo + MoveIt) |
| 워크스페이스 | `~/study/robot_vision` (= GitHub 저장소 = colcon 워크스페이스 루트) |

---

## Step 0. conda 자동 활성화 끄기

conda `base` 가 켜져 있으면 `python3` 가 conda 의 Python(3.11 등)으로 잡혀서 ROS2 의 `rclpy`(3.12 용)를 불러오지 못하고,
`colcon build` 결과물도 엉뚱한 Python 으로 만들어집니다.

```bash
conda config --set auto_activate_base false
# 새 터미널을 열고 확인: /usr/bin/python3, Python 3.12.x 가 나와야 함
which python3 && python3 --version
```

다른 과제에서 conda 가 필요하면 그때만 `conda activate <env>` 하면 됩니다.

## Step 1. ROS2 Jazzy 설치

```bash
# 1. 로케일
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Universe 저장소 + ROS2 apt 소스 (ros2-apt-source 패키지 방식, 현재 공식 문서 방식)
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# 3. ROS2 Jazzy Desktop + 개발 도구(colcon 포함) + cv_bridge
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-cv-bridge

# 4. 설치 확인 (터미널 2개)
source /opt/ros/jazzy/setup.bash && ros2 run demo_nodes_cpp talker
source /opt/ros/jazzy/setup.bash && ros2 run demo_nodes_py listener
```

> 예전 방식(`ros.key` 를 curl 로 받아 `/etc/apt/sources.list.d/ros2.list` 직접 작성)으로 이미 설치했다면
> 2025년 서명 키 교체 이후 `apt update` 에서 서명 오류가 날 수 있습니다. 그럴 땐 `ros2.list` 와
> `/usr/share/keyrings/ros-archive-keyring.gpg` 를 지우고 위 2번을 다시 하면 됩니다.

`~/.bashrc` 에 `source /opt/ros/jazzy/setup.bash` 를 넣지 않고, Step 6 의 `env.sh` 로 한 번에 켭니다.

## Step 2. Python 패키지

```bash
# OpenCV·NumPy 는 apt 로 (ROS2 의 cv_bridge 와 같은 NumPy 버전을 써야 함)
sudo apt install -y python3-opencv python3-numpy python3-yaml python3-venv python3-pytest

# 저장소 받기
git clone https://github.com/robot-vision-arms/vision-arm.git ~/study/robot_vision
cd ~/study/robot_vision

# 시스템 패키지(rclpy, cv2, numpy)를 그대로 보면서 PySide6 만 추가로 설치하는 venv
python3 -m venv --system-site-packages .venv
touch .venv/COLCON_IGNORE          # colcon 이 .venv 안을 패키지로 착각하지 않게
source .venv/bin/activate
pip install -r requirements.txt

# 확인: 셋 다 에러 없이 출력돼야 함
python -c "import rclpy, cv2, numpy, PySide6; print(cv2.__version__, numpy.__version__, PySide6.__version__)"
```

## Step 3. 시뮬레이터 패키지 (rosdep)

Gazebo·UR5e·MoveIt 관련 패키지는 `src/ur5e_pick_place/package.xml` 에 의존성으로 적어 두었고, **rosdep 이 그걸 읽어 apt 로 설치**합니다.
목록을 문서와 package.xml 두 곳에 따로 적으면 어긋나기 쉬워서, 기준은 package.xml 하나로 둡니다.

```bash
sudo apt install -y python3-rosdep
sudo rosdep init            # 처음 한 번만. "already exists" 가 나오면 이미 된 것
rosdep update

cd ~/study/robot_vision
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
```

rosdep 이 설치하는 주요 패키지 (확인용):

| apt 패키지 | 용도 |
|---|---|
| `ros-jazzy-ur-simulation-gz` | UR 로봇 Gazebo 시뮬레이션 (`ur_sim_control.launch.py`) |
| `ros-jazzy-ur-description`, `ros-jazzy-ur-moveit-config` | UR5e 모델, MoveIt 설정 (`ur_moveit.launch.py`) |
| `ros-jazzy-moveit` | MoveIt2 (`move_group`) |
| `ros-jazzy-pymoveit2` | Python 에서 MoveIt 부르기 |
| `ros-jazzy-ros-gz-sim`, `ros-jazzy-ros-gz-bridge` | Gazebo Harmonic 실행·스폰, ROS ↔ Gazebo 토픽 다리 (Gazebo 본체도 같이 설치됨) |

rosdep 이 어떤 키를 못 찾으면 위 표의 패키지를 `sudo apt install -y <이름>` 으로 직접 설치하고, 무엇이 안 됐는지 [DECISIONS.md](DECISIONS.md) 4장 표에 적어 주세요.

> pymoveit2 를 pip 로 따로 설치하지 마세요. apt 판과 겹치면 어느 쪽이 import 될지 꼬입니다.

## Step 4. 빌드

패키지 뼈대(`src/vision_pick_place`, `src/ur5e_pick_place`)는 저장소에 이미 있으므로 `ros2 pkg create` 는 다시 하지 않습니다.

```bash
source ~/study/robot_vision/env.sh
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

뼈대 상태에서는 아직 구현 안 된 함수의 테스트가 `skipped` 로 나옵니다. 구현하면서 `@pytest.mark.skip` 줄을 지웁니다.

`--symlink-install` 을 쓰면 Python 파일을 고친 뒤 다시 빌드하지 않아도 바로 반영됩니다
(`setup.py`, `package.xml`, launch/config 파일, `models/` 폴더를 추가했을 때만 다시 빌드).

## Step 5. 시뮬레이터 구동 확인 (1주차 확인 ①)

**1주차 안에 3명 모두** 해 봅니다. 여기서 막히면 일정 전체가 흔들려서, 문제를 빨리 찾으려고 앞에 둡니다 (PROJECT.md 8장).

### 5-1. WSL 메모리

```bash
free -h        # total 이 8 GB 미만이면 아래처럼 늘린다
```

Windows 의 `%UserProfile%\.wslconfig`:

```ini
[wsl2]
memory=8GB
```

PowerShell 에서 `wsl --shutdown` 후 WSL 을 다시 엽니다.

### 5-2. UR 기본 예제부터 (우리 파일 없이)

```bash
source ~/study/robot_vision/env.sh
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
```

Gazebo 창에 UR5e 가 서 있고 RViz 가 뜨면, RViz 의 MotionPlanning 패널에서 목표 자세를 끌어다 놓고 **Plan & Execute** → Gazebo 속 팔이 움직이면 통과.

### 5-3. 우리 런치 파일 인자 확인

`sim.launch.py` 는 UR 런치 파일 두 개를 직접 include 하면서 `description_file`, `world_file`, `gazebo_gui`, `launch_rviz` 인자를 넘깁니다.
버전에 따라 인자 이름이 다를 수 있어 직접 확인합니다:

```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py --show-args
ros2 launch ur_moveit_config ur_moveit.launch.py --show-args
ls $(ros2 pkg prefix ur_simulation_gz)/share/ur_simulation_gz/urdf/     # ur_gz.urdf.xacro 가 있는지 (흡착 패드 xacro 가 include)
```

다르면 `sim.launch.py`, `ur5e_vacuum.urdf.xacro` 를 고치고, 결과를 [DECISIONS.md](DECISIONS.md) 4장 표에 적습니다. 그다음:

```bash
ros2 launch ur5e_pick_place sim.launch.py
```

### 5-4. 화면이 안 나오거나 너무 느릴 때

| 증상 | 해볼 것 |
|---|---|
| Gazebo 창이 검거나 깨짐 | `env.sh` 의 `export LIBGL_ALWAYS_SOFTWARE=1` 주석 해제 (소프트웨어 렌더링, 느려짐) |
| Gazebo 창이 너무 느림 | `ros2 launch ur5e_pick_place sim.launch.py gazebo_gui:=false` → Gazebo 는 화면 없이 돌고 RViz 로만 확인 |
| 팔이 궤적을 못 따라감 | `config/sim.yaml` 의 `max_velocity_scaling` 을 더 낮춤 |
| 메모리 부족으로 꺼짐 | 5-1 |

그래도 안 되면 PROJECT.md 8장 "실패하면" 대로 범위를 줄입니다.

## Step 6. 매일 작업 시작할 때

```bash
source ~/study/robot_vision/env.sh     # conda 해제 → ROS2 → venv → install/setup.bash 순서로 켜고 저장소로 이동
code .                         # VS Code (WSL 확장)
```

실행 (과제 필수 범위):

```bash
ros2 run vision_pick_place robot_server          # 터미널 1: 가상 로봇 TCP 서버
ros2 launch vision_pick_place pipeline.launch.py # 터미널 2: 비전 노드 + 브리지 + GUI
```

실행 (Gazebo 확장까지):

```bash
ros2 launch ur5e_pick_place sim.launch.py                        # 터미널 1: Gazebo + UR5e + MoveIt + RViz
ros2 run vision_pick_place robot_server --publish-ros            # 터미널 2: TCP 서버가 받은 패킷을 /robot/targets 로 넘김
ros2 launch vision_pick_place pipeline.launch.py                 # 터미널 3: 비전 + 브리지 + GUI
ros2 service call /sim/run_cycle std_srvs/srv/Trigger            # 터미널 4: 지금 보이는 부품으로 한 사이클
```

## Step 7. WSL2 에서 USB 웹캠 쓰기

WSL2 는 USB 장치를 기본으로 못 봅니다. Windows 쪽에 [usbipd-win](https://github.com/dorssel/usbipd-win) 을 설치하고 연결합니다.

```powershell
# Windows PowerShell (관리자)
winget install usbipd
usbipd list                          # 웹캠 BUSID 확인 (예: 2-3)
usbipd bind --busid 2-3
usbipd attach --wsl --busid 2-3      # 웹캠을 뽑았다 꽂거나 재부팅하면 attach 다시
```

```bash
# WSL
sudo modprobe uvcvideo
ls /dev/video*                       # /dev/video0 이 보이면 성공
```

웹캠이 끝내 안 잡히면 촬영은 Windows 에서 하고 이미지 폴더를 `params.yaml` 의 `image_dir` 로 지정해 진행합니다.
**1주차 안에 각자 확인**해 두세요.

---

## 원본 가이드에서 바꾼 점

| # | 원본 | 변경 | 이유 |
|---|---|---|---|
| 1 | conda `python=3.10` 가상환경에 설치 | 시스템 Python 3.12 + `venv --system-site-packages` | ROS2 Jazzy 의 `rclpy` 는 Ubuntu 24.04 시스템 Python **3.12** 용으로 빌드된 바이너리라 3.10 환경에서는 `import rclpy` 가 실패합니다. conda base(현재 3.11)가 자동으로 켜져 있어도 같은 문제가 납니다. |
| 2 | `pip install opencv-python numpy` | `apt install python3-opencv python3-numpy` | pip 최신 opencv/numpy 는 NumPy 2.x 를 끌어오는데, apt 의 `cv_bridge` 는 NumPy 1.26 기준으로 빌드돼 있어 충돌할 수 있습니다. 또 `opencv-python` 휠은 자체 Qt 를 포함해서 PySide6 와 Qt 플러그인 충돌을 일으키는 경우가 많습니다. |
| 3 | `ros.key` curl + `ros2.list` 수동 작성 | `ros2-apt-source` 패키지 | 2025년 ROS 서명 키 교체 이후 공식 설치 문서가 이 방식으로 바뀌었습니다. 키가 바뀌어도 apt 업데이트로 따라갑니다. |
| 4 | `python3-colcon-common-extensions` | `ros-dev-tools` | colcon 을 포함한 ROS 개발 도구 묶음으로, 현재 공식 문서 기준입니다. |
| 5 | `echo "source ..." >> ~/.bashrc` 2회 | `env.sh` 를 필요할 때 `source` | 명령을 두 번 실행하면 `.bashrc` 에 줄이 중복되고, 빌드 전에 `install/setup.bash` 를 source 하면 터미널을 열 때마다 에러가 납니다. conda 해제·venv 활성화 순서도 한 곳에서 관리합니다. |
| 6 | 워크스페이스 `~/study/robot_vision` 를 새로 만들고 git 위치는 정하지 않음 | `~/study/robot_vision` 자체를 git 저장소로 (GitHub 저장소를 이 폴더로 clone) | 팀원이 clone 하면 바로 빌드되게 했습니다. `build/ install/ log/` 는 `.gitignore` 에 있습니다. |
| 7 | 의존성 `rclpy std_msgs sensor_msgs geometry_msgs` | + `cv_bridge` | 디버그 영상(`sensor_msgs/Image`)을 GUI 로 보내려면 OpenCV 이미지 ↔ ROS 메시지 변환이 필요합니다. |
| 8 | 파일 3개 (`camera_stream`, `vision_detector`, `calibrator`) | + ROS 노드, TCP 브리지·가상 로봇 서버, GUI, KPI 스크립트 | 과제 필수 항목(TCP 송수신, GUI 4화면, KPI 측정)을 담을 파일이 없었습니다. 구조는 `docs/PROJECT.md` 5장 참고. |
| 9 | 파일마다 대원 A/B/C 고정 지정 | 주차마다 주 담당을 돌림 (PROJECT.md 2.3) | 팀 원칙이 "전원이 전 과정을 학습" 이라 고정 소유권과 맞지 않습니다. 작업 묶음은 받은 분담안 것을 쓰되 사람을 돌립니다. |
| 10 | MuJoCo 를 주요 라이브러리로 명시, 설치 명령은 없음 | MuJoCo 제거, 시뮬레이터는 Gazebo Harmonic + `ur_simulation_gz` (Step 3) | 받은 역할 분담안이 Gazebo·UR5e·MoveIt2 기준이고, UR 공식 시뮬레이션 패키지와 MoveIt 설정이 Jazzy 용으로 apt 에 있어서 바로 쓸 수 있습니다. 과제 필수 범위(TCP 서버)는 그대로 유지합니다. |
| 11 | 웹캠 연결 언급 없음 | Step 7 추가 | WSL2 는 기본적으로 USB 웹캠이 보이지 않습니다 (`/dev/video*` 없음). |
| 12 | `ur_simulation_gazebo` 클론 | `ros-jazzy-ur-simulation-gz` apt 설치 | `ur_simulation_gazebo` 는 Gazebo Classic 용(Humble)입니다. Jazzy 의 짝은 Gazebo Harmonic 이고 그쪽 패키지가 `ur_simulation_gz` 입니다. apt 판이 있어 클론·빌드할 필요가 없습니다. |
| 13 | 의존성 직접 나열 | `rosdep install` (Step 3) | 의존성 기준을 package.xml 한 곳으로 둡니다. 문서와 따로 적으면 어긋납니다. |
| 14 | 시뮬레이터 구동 확인 시점 없음 (3단계에서 처음 구축) | 1주차 Step 5 | WSL2 에서 Gazebo 렌더링·메모리 문제는 늦게 알면 대응할 시간이 없습니다. |
