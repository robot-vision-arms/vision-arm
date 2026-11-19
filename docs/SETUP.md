# 개발 환경 구축 가이드 (WSL2 Ubuntu 24.04 + ROS2 Jazzy + PySide6 + Gazebo Harmonic + MoveIt2)

팀원 3명이 같은 순서로 따라 하면 같은 환경이 되도록 정리한 문서입니다. 순서만 빠르게 따라가려면 맨 아래 [부록 체크리스트](#부록-1주차-설치-체크리스트-실행-순서)를 보세요.
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
| 모델 비교 (선택) | scikit-learn, PyTorch (CPU 판), Ultralytics YOLO. ROS 용 `.venv` 와 **분리된** `.venv-ml` 에 설치 (Step 8). GPU 없이 CPU 로 학습·추론 |
| 워크스페이스 | `~/vision-arm` (= GitHub 저장소 = colcon 워크스페이스 루트) |

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
git clone https://github.com/robot-vision-arms/vision-arm.git ~/vision-arm
cd ~/vision-arm

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

cd ~/vision-arm
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
```

rosdep 이 설치하는 주요 패키지 (확인용):

| apt 패키지 | 용도 |
|---|---|
| `ros-jazzy-ur-simulation-gz` | UR 로봇 Gazebo 시뮬레이션 (`ur_sim_control.launch.py`) |
| `ros-jazzy-ur-description`, `ros-jazzy-ur-moveit-config`, `ros-jazzy-ur-robot-driver` | UR5e 모델, MoveIt 설정 (`ur_moveit.launch.py`) |
| `ros-jazzy-moveit` | MoveIt2 (`move_group`) |
| `ros-jazzy-pymoveit2` | Python 에서 MoveIt 부르기 |
| `ros-jazzy-ros-gz-sim`, `ros-jazzy-ros-gz-bridge` | Gazebo Harmonic 실행·스폰, ROS ↔ Gazebo 토픽 다리 (Gazebo 본체도 같이 설치됨) |

rosdep 이 어떤 키를 못 찾으면 위 표의 패키지를 `sudo apt install -y <이름>` 으로 직접 설치하고, 무엇이 안 됐는지 [DECISIONS.md](DECISIONS.md) 4장 표에 적어 주세요.

> pymoveit2 를 pip 로 따로 설치하지 마세요. apt 판과 겹치면 어느 쪽이 import 될지 꼬입니다.

## Step 4. 빌드

패키지 뼈대(`src/vision_pick_place`, `src/ur5e_pick_place`)는 저장소에 이미 있으므로 `ros2 pkg create` 는 다시 하지 않습니다.

```bash
source ~/vision-arm/env.sh
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
memory=10GB
swap=8GB
```

값은 Windows 전체 메모리 16 GB 기준입니다 (Windows 쪽에 약 6 GB 를 남김, 부록 1단계). 16 GB 보다 작은 PC 는 `memory=8GB` 로 둡니다.
PowerShell 에서 `wsl --shutdown` 후 WSL 을 다시 엽니다. 파일이 `.wslconfig.txt` 로 저장되지 않았는지 확인하세요.

### 5-2. UR 기본 예제부터 (우리 파일 없이)

```bash
source ~/vision-arm/env.sh
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
source ~/vision-arm/env.sh     # conda 해제 → ROS2 → venv → install/setup.bash 순서로 켜고 저장소로 이동
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

## Step 7. 스마트폰을 카메라로 쓰기

PC 에 웹캠이 없으므로 스마트폰을 IP 카메라로 씁니다. 영상은 Wi-Fi 로 받기 때문에 USB 연결(usbipd)은 필요 없습니다.

1. 폰에 IP 카메라 앱을 설치합니다 (Android 예: IP Webcam. iPhone 은 HTTP(MJPEG)·RTSP 영상 주소를 주는 앱).
   앱에서 서버를 시작하면 `http://192.168.0.23:8080` 같은 주소가 화면에 나옵니다.
2. PC 와 폰을 **같은 Wi-Fi** 에 연결합니다.
3. 영상 주소를 `src/vision_pick_place/config/params.yaml` 의 `camera_url` 에 넣습니다.
   IP Webcam 은 보통 주소 끝에 `/video` 를 붙입니다. 앱마다 경로가 다르니 앱 화면의 안내를 따릅니다.
4. WSL 에서 확인합니다.

```bash
source ~/vision-arm/env.sh
python scripts/check_environment.py        # '스마트폰 카메라 접속' 이 OK 인지
python -c "import cv2; c=cv2.VideoCapture('http://192.168.0.23:8080/video'); ok,f=c.read(); print(ok, None if f is None else f.shape)"
```

   `True (720, 1280, 3)` 처럼 나오면 통과입니다 (주소는 자기 폰 것으로).
5. 폰을 거치대로 작업면 위에 고정합니다. 보정(ChArUco 보드·작업 시트)과 부품 인식을 같은 위치·해상도로 해야 px→mm 가 맞습니다.
   앱에서 해상도를 고정하고, 가능하면 자동 초점·자동 노출·화면 회전을 끕니다. 화면 꺼짐을 막고 충전기를 연결해 둡니다.

접속이 안 될 때:

- 먼저 Windows 브라우저로 같은 주소를 열어 봅니다. 여기서도 안 되면 폰·Wi-Fi 문제입니다.
- 학교·회사 Wi-Fi 는 기기끼리 통신을 막는 경우가 있습니다. 폰 핫스팟에 PC 를 연결해 봅니다.
- Windows 에서는 되는데 WSL 에서만 안 되면 Windows 방화벽과 WSL 네트워크 설정을 확인합니다.
- 폰 IP 는 Wi-Fi 에 다시 붙으면 바뀔 수 있습니다. 바뀌면 `camera_url` 을 고칩니다.
- 끝내 안 되면 폰으로 사진·동영상을 찍어 그 폴더를 `params.yaml` 의 `image_dir` 로 지정해 진행합니다.

**1주차 안에 각자 확인**해 두세요.

## Step 8. (선택) 모델 비교용 학습 환경

검출·분류 모델 비교(DECISIONS.md 11절, 3·4주차 팀원 1 담당)를 할 PC 에서만 합니다. 다른 팀원은 건너뛰어도 됩니다.

ROS 용 `.venv` 에 설치하지 않습니다. torch·ultralytics 는 pip 판 `opencv-python` 과 NumPy 2.x 를 끌어오는데,
그러면 apt 의 `cv_bridge`(NumPy 1.26 빌드)와 PySide6 의 Qt 가 꼬일 수 있습니다 (아래 "원본 가이드에서 바꾼 점" 2번과 같은 이유).
그래서 시스템 패키지를 보지 않는 **별도 venv** 를 만듭니다.

```bash
cd ~/vision-arm
python3 -m venv .venv-ml                 # --system-site-packages 를 붙이지 않음
touch .venv-ml/COLCON_IGNORE
source .venv-ml/bin/activate             # env.sh 대신 이걸 켬 (ROS 는 안 씀)

pip install --upgrade pip
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu   # GPU 가 없으므로 CPU 판 (용량이 작음)
pip install ultralytics scikit-learn opencv-python-headless

python -c "import torch, ultralytics, sklearn, cv2; print(torch.__version__, torch.cuda.is_available(), ultralytics.__version__, sklearn.__version__, cv2.__version__)"
deactivate
```

- 마지막 줄이 에러 없이 버전을 출력하고 `torch.cuda.is_available()` 가 `False` 면 통과 (CPU 학습).
- 학습 결과(가중치 `*.pt`)와 변환한 학습 데이터는 `data/models/` 에 둡니다. git 에 올라가지 않습니다.
- 학습이 너무 느리면 Google Colab 무료 GPU 로 학습만 하고, 받은 가중치를 이 PC 에서 CPU 로 추론합니다.
- 비교가 끝나 학습 모델을 `vision_node` 에 넣기로 하면, 그때 ROS 환경과 함께 쓰는 방법(ONNX 로 내보내 OpenCV DNN 으로 추론 등)을 따로 정합니다.

---

## 원본 가이드에서 바꾼 점

| # | 원본 | 변경 | 이유 |
|---|---|---|---|
| 1 | conda `python=3.10` 가상환경에 설치 | 시스템 Python 3.12 + `venv --system-site-packages` | ROS2 Jazzy 의 `rclpy` 는 Ubuntu 24.04 시스템 Python **3.12** 용으로 빌드된 바이너리라 3.10 환경에서는 `import rclpy` 가 실패합니다. conda base(현재 3.11)가 자동으로 켜져 있어도 같은 문제가 납니다. |
| 2 | `pip install opencv-python numpy` | `apt install python3-opencv python3-numpy` | pip 최신 opencv/numpy 는 NumPy 2.x 를 끌어오는데, apt 의 `cv_bridge` 는 NumPy 1.26 기준으로 빌드돼 있어 충돌할 수 있습니다. 또 `opencv-python` 휠은 자체 Qt 를 포함해서 PySide6 와 Qt 플러그인 충돌을 일으키는 경우가 많습니다. |
| 3 | `ros.key` curl + `ros2.list` 수동 작성 | `ros2-apt-source` 패키지 | 2025년 ROS 서명 키 교체 이후 공식 설치 문서가 이 방식으로 바뀌었습니다. 키가 바뀌어도 apt 업데이트로 따라갑니다. |
| 4 | `python3-colcon-common-extensions` | `ros-dev-tools` | colcon 을 포함한 ROS 개발 도구 묶음으로, 현재 공식 문서 기준입니다. |
| 5 | `echo "source ..." >> ~/.bashrc` 2회 | `env.sh` 를 필요할 때 `source` | 명령을 두 번 실행하면 `.bashrc` 에 줄이 중복되고, 빌드 전에 `install/setup.bash` 를 source 하면 터미널을 열 때마다 에러가 납니다. conda 해제·venv 활성화 순서도 한 곳에서 관리합니다. |
| 6 | 워크스페이스 `~/vision-arm` 를 새로 만들고 git 위치는 정하지 않음 | `~/vision-arm` 자체를 git 저장소로 (GitHub 저장소를 이 폴더로 clone) | 팀원이 clone 하면 바로 빌드되게 했습니다. `build/ install/ log/` 는 `.gitignore` 에 있습니다. |
| 7 | 의존성 `rclpy std_msgs sensor_msgs geometry_msgs` | + `cv_bridge` | 디버그 영상(`sensor_msgs/Image`)을 GUI 로 보내려면 OpenCV 이미지 ↔ ROS 메시지 변환이 필요합니다. |
| 8 | 파일 3개 (`camera_stream`, `vision_detector`, `calibrator`) | + ROS 노드, TCP 브리지·가상 로봇 서버, GUI, KPI 스크립트 | 과제 필수 항목(TCP 송수신, GUI 4화면, KPI 측정)을 담을 파일이 없었습니다. 구조는 `docs/PROJECT.md` 5장 참고. |
| 9 | 파일마다 대원 A/B/C 고정 지정 | 주차마다 주 담당을 돌림 (PROJECT.md 2.3) | 팀 원칙이 "전원이 전 과정을 학습" 이라 고정 소유권과 맞지 않습니다. 작업 묶음은 받은 분담안 것을 쓰되 사람을 돌립니다. |
| 10 | MuJoCo 를 주요 라이브러리로 명시, 설치 명령은 없음 | MuJoCo 제거, 시뮬레이터는 Gazebo Harmonic + `ur_simulation_gz` (Step 3) | 받은 역할 분담안이 Gazebo·UR5e·MoveIt2 기준이고, UR 공식 시뮬레이션 패키지와 MoveIt 설정이 Jazzy 용으로 apt 에 있어서 바로 쓸 수 있습니다. 과제 필수 범위(TCP 서버)는 그대로 유지합니다. |
| 11 | 웹캠 연결 언급 없음 | Step 7 스마트폰 IP 카메라 | PC 에 웹캠이 없고, WSL2 는 USB 장치를 기본으로 못 봅니다. 폰 영상은 네트워크로 받으므로 usbipd 가 필요 없습니다. |
| 12 | `ur_simulation_gazebo` 클론 | `ros-jazzy-ur-simulation-gz` apt 설치 | `ur_simulation_gazebo` 는 Gazebo Classic 용(Humble)입니다. Jazzy 의 짝은 Gazebo Harmonic 이고 그쪽 패키지가 `ur_simulation_gz` 입니다. apt 판이 있어 클론·빌드할 필요가 없습니다. |
| 13 | 의존성 직접 나열 | `rosdep install` (Step 3) | 의존성 기준을 package.xml 한 곳으로 둡니다. 문서와 따로 적으면 어긋납니다. |
| 14 | 시뮬레이터 구동 확인 시점 없음 (3단계에서 처음 구축) | 1주차 Step 5 | WSL2 에서 Gazebo 렌더링·메모리 문제는 늦게 알면 대응할 시간이 없습니다. |

---

## 부록. 1주차 설치 체크리스트 (실행 순서)

WEEK1.md 에 있던 설치 순서 체크리스트를 옮겨 왔다 (2026-11-19). 명령의 이유와 자세한 설명은 위 Step 들에 있고,
여기는 "지금 무엇을 하고, 무엇이 나오면 다음으로 넘어가는지" 만 적는다. 팀원별 통과 여부는 [WEEK1.md 1절](WEEK1.md#1-현황-요약-2026-11-19) 표에 적는다.
`sudo` 는 비밀번호를 묻기 때문에, 명령은 **각자 WSL 터미널에서 직접** 실행한다.

### 전체 순서

```
[Windows]  1. WSL 메모리 늘리기 ──► 2. 폰 IP 카메라 앱 설치
[WSL]      3. conda 끄기 ──► 4. ROS2 Jazzy ──► 5. Python·venv ──► 6. 시뮬레이터 패키지 (rosdep)
           ──► 7. 빌드·테스트 ──► 8. 시뮬레이터 확인 ① ──► 9. 폰 영상 수신 ──► 10. 결과 기록
```

순서를 지켜야 하는 곳: **3 → 4 → 5** (conda 가 켜진 채로 venv 를 만들면 Python 3.11 venv 가 생김),
**4 → 6** (rosdep 은 ROS2 가 있어야 동작), **5·6 → 7** (`env.sh` 가 `.venv` 와 ROS2 를 같이 켬).
1·2 는 Windows 쪽이라 WSL 작업 전에 끝내 둡니다.

### 1. WSL 메모리 늘리기 (Windows)

Gazebo + MoveIt + RViz 를 같이 띄우면 7 GB 로는 부족할 수 있습니다 (권장 8 GB 이상, SETUP.md 5-1).
이 PC 는 Windows 전체가 15.7 GB 라 **10 GB** 로 두고, Windows 쪽에 약 6 GB 를 남깁니다.
(10 GB 는 "Windows 가 버틸 만큼 남기면서 8 GB 권장보다 여유" 로 고른 값이고, 실제로 돌려서 확인한 값은 아닙니다.
Windows 가 느려지면 8 GB 로 내립니다.)

메모장으로 `C:\Users\user1\.wslconfig` 파일을 새로 만들어 저장합니다 (확장자 `.txt` 가 붙지 않게 주의):

```ini
[wsl2]
memory=10GB
swap=8GB
```

```powershell
# PowerShell
wsl --shutdown
```

WSL 을 다시 열고:

```bash
free -h      # Mem total 이 9 GB 이상 보이면 통과 (커널 예약분 때문에 10 보다 조금 작게 나옴)
```

### 2. 폰 IP 카메라 앱 설치

영상 수신 확인은 9단계에서 하지만, 앱 설치와 거치대 준비는 미리 해 둡니다 (SETUP.md Step 7).

- [ ] 폰에 IP 카메라 앱 설치 (Android 예: IP Webcam)
- [ ] 앱에서 서버를 시작하고, 같은 Wi-Fi 의 Windows 브라우저로 앱 주소를 열었을 때 영상이 보이면 통과
- [ ] 작업면을 위에서 내려다보게 폰을 고정할 거치대 준비

### 3. conda 자동 활성화 끄기

```bash
conda config --set auto_activate_base false
```

터미널을 **닫고 새로 연 뒤**:

```bash
which python3 && python3 --version
```

- [ ] `/usr/bin/python3`, `Python 3.12.x` 가 나오면 통과

> 다른 과제에서 conda 가 필요하면 그때 `conda activate <env>` 로 켜면 됩니다. 이 프로젝트의 `env.sh` 는 conda 가 켜져 있어도 먼저 끕니다.

### 4. ROS2 Jazzy 설치

SETUP.md Step 1 의 명령을 그대로 순서대로 실행합니다 (로케일 → apt 소스 → `ros-jazzy-desktop ros-dev-tools ros-jazzy-cv-bridge`).
다운로드 양이 많아서 시간이 꽤 걸립니다.

확인 (터미널 2개):

```bash
source /opt/ros/jazzy/setup.bash && ros2 run demo_nodes_cpp talker     # 터미널 A
source /opt/ros/jazzy/setup.bash && ros2 run demo_nodes_py listener    # 터미널 B
```

- [ ] 터미널 B 에 `I heard: [Hello World: N]` 가 계속 찍히면 통과

### 5. Python 패키지·venv

이 PC 에는 저장소가 이미 `~/vision-arm` 에 있으니 `git clone` 은 건너뜁니다 (다른 팀원은 SETUP.md Step 2 대로 clone).

```bash
sudo apt install -y python3-opencv python3-numpy python3-yaml python3-venv python3-pytest

cd ~/vision-arm
python3 -m venv --system-site-packages .venv
touch .venv/COLCON_IGNORE
source .venv/bin/activate
pip install -r requirements.txt          # PySide6, pytest 만 설치됨

source /opt/ros/jazzy/setup.bash
python -c "import rclpy, cv2, numpy, PySide6; print(cv2.__version__, numpy.__version__, PySide6.__version__)"
```

- [ ] 마지막 줄이 에러 없이 버전 3개를 출력하면 통과
- [ ] 출력된 버전을 10단계 표에 적어 둠

### 6. 시뮬레이터 패키지 (rosdep)

Gazebo Harmonic, UR5e, MoveIt2, pymoveit2 를 `src/ur5e_pick_place/package.xml` 기준으로 한 번에 설치합니다 (SETUP.md Step 3).

```bash
sudo apt install -y python3-rosdep
sudo rosdep init            # "already exists" 가 나오면 이미 된 것이라 넘어감
rosdep update

cd ~/vision-arm
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
```

- [ ] 마지막에 `#All required rosdeps installed successfully` 가 나오면 통과
- [ ] 못 찾는 키가 있으면 SETUP.md Step 3 표의 패키지를 `sudo apt install -y <이름>` 으로 직접 설치하고 10단계에 기록

### 7. 빌드·테스트

```bash
source ~/vision-arm/env.sh      # 이제부터는 매번 이 한 줄로 시작
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

- [ ] `colcon build` 가 `2 packages finished` 로 끝나면 통과
- [ ] 테스트는 실패(failure) 0 이면 통과. 뼈대라서 `skipped` 가 여러 개 나오는 것은 정상

### 8. 시뮬레이터 구동 확인 ① (Go / No-Go)

**8-1. UR 기본 예제** (우리 파일 없이):

```bash
source ~/vision-arm/env.sh
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
```

- [ ] Gazebo 창에 UR5e 가 서 있음
- [ ] RViz MotionPlanning 패널에서 목표 자세를 끌어 놓고 **Plan & Execute** → Gazebo 속 팔이 움직임

**8-2. 런치 인자 확인** (우리 `sim.launch.py` 가 넘기는 인자가 실제로 있는지):

```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py --show-args   # description_file, world_file, gazebo_gui, launch_rviz
ros2 launch ur_moveit_config ur_moveit.launch.py --show-args        # ur_type, use_sim_time, launch_rviz
ls $(ros2 pkg prefix ur_simulation_gz)/share/ur_simulation_gz/urdf/ # ur_gz.urdf.xacro
```

- [ ] 인자·파일이 모두 있으면 통과. 다르면 `sim.launch.py` / `ur5e_vacuum.urdf.xacro` 를 고치고 10단계에 기록

**8-3. 우리 런치 파일:**

```bash
ros2 launch ur5e_pick_place sim.launch.py
```

- [ ] 작업 매트와 상자 2개가 있는 월드에 UR5e 가 뜨고, RViz 에서 Plan & Execute 로 움직이면 **확인 ① 통과**

화면이 검거나 너무 느리면 SETUP.md 5-4 표 순서대로 (소프트웨어 렌더링 → `gazebo_gui:=false`).
그래도 안 되면 PROJECT.md 8장 "실패하면" 대로 진행하고, 팀에 바로 알립니다.

### 9. 폰 영상 수신

SETUP.md Step 7 대로 `params.yaml` 의 `camera_url` 에 폰 영상 주소를 넣고:

```bash
source ~/vision-arm/env.sh
python scripts/check_environment.py        # '스마트폰 카메라 접속' 이 OK
python -c "import cv2; c=cv2.VideoCapture('<camera_url>'); ok,f=c.read(); print(ok, None if f is None else f.shape)"
```

- [ ] 점검 스크립트의 `스마트폰 카메라 접속` 이 OK
- [ ] 위 명령이 `True (높이, 너비, 3)` 을 출력하면 통과
- [ ] 끝내 안 되면 폰으로 찍은 사진·동영상 폴더를 `params.yaml` 의 `image_dir` 로 쓰기로 하고 팀에 알림

보유 MVTec 사진에는 실제 mm 정답이 없으므로 위치 KPI를 대신하지 않습니다.

실시간 뷰어 사용법은 [WEEK1.md 2절](WEEK1.md#2-실행-방법) 에 있다.

### 10. 결과 기록

설치가 끝나면 아래를 채웁니다. 팀원 3명의 값이 다르면 그 차이가 나중에 "내 PC 에서는 되는데" 문제의 원인이 됩니다.

| 기록할 것 | 어디에 |
|---|---|
| 8-2 런치 인자·xacro 경로 확인 결과, rosdep 이 못 찾은 키 | [DECISIONS.md 4장](DECISIONS.md#4-아직-확인-못-한-것-12주차에-채움) 표 |
| 확인 ① 통과 여부, 쓴 대체 방법 (소프트웨어 렌더링 등) | DECISIONS.md 4장 "WSL2 에서 Gazebo + MoveIt + RViz" 행 |
| 버전: PySide6·OpenCV·NumPy (5단계), `gz sim --version`, `apt-cache policy ros-jazzy-moveit` | PROJECT.md 7장 측정 장비 표 |
| PC 사양: CPU·RAM·GPU·WSL 메모리 설정 | PROJECT.md 7장 측정 장비 표 |

환경 설치가 끝나면 1주차 나머지 일(폰 고정, ChArUco 캘리브레이션, 작업 시트 호모그래피)로 넘어갑니다 (PROJECT.md 8장).
