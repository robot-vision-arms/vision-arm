# 1주차 환경 설치 체크리스트

1주차 환경 설치를 **실행 순서대로** 정리한 문서입니다. 명령의 이유와 자세한 설명은 [SETUP.md](SETUP.md) 에 있고,
이 문서는 "지금 무엇을 하고, 무엇이 나오면 다음으로 넘어가는지" 만 적습니다.
1주차 전체 목표(캘리브레이션 포함)는 [PROJECT.md 8장](PROJECT.md#8-4주-로드맵) 에 있습니다.

`sudo` 는 비밀번호를 묻기 때문에, 명령은 **각자 WSL 터미널에서 직접** 실행합니다.

## 0. 이 PC 의 최초 설치 전 상태 (2026-11-10 확인)

아래 표는 설치 전 기록입니다. 최신 설치 상태는 문서 마지막에 기록합니다. 팀원마다 각자 확인합니다.

| 항목 | 확인 명령 | 이 PC 값 | 필요한 조치 |
|---|---|---|---|
| Ubuntu | `lsb_release -ds` | 24.04.5 LTS | 없음 |
| WSL | (PowerShell) `wsl --version` | 2.7.12, WSLg 1.0.73 | 없음 |
| WSL 메모리 | `free -h` | **7 GB** (Windows 전체 15.7 GB 의 기본 절반) | **1단계**: 10 GB 로 늘림 |
| `.wslconfig` | (PowerShell) `Test-Path $env:USERPROFILE\.wslconfig` | 없음 | 1단계에서 새로 만듦 |
| GPU | (PowerShell) `(Get-CimInstance Win32_VideoController).Name` | Intel UHD Graphics 730 (내장) | 8단계에서 Gazebo 화면 확인. 느리면 SETUP.md 5-4 |
| 디스크 여유 | `df -h ~` | 861 GB | 없음 (15 GB 이상이면 됨) |
| ROS2 | `ls /opt/ros` | **없음** | 4단계 |
| python3 | `which python3 && python3 --version` | **conda base 3.11** (`/home/user1/anaconda3/bin/python3`) | **3단계**: conda 자동 활성화 끄기 |
| 시스템 Python | `/usr/bin/python3 --version` | 3.12.3 | 없음 (ROS2 Jazzy 짝) |
| 웹캠 | `ls /dev/video*` | **없음** | 9단계 (usbipd) |

## 전체 순서

```
[Windows]  1. WSL 메모리 늘리기 ──► 2. usbipd 설치 (웹캠용, 미리 받아 둠)
[WSL]      3. conda 끄기 ──► 4. ROS2 Jazzy ──► 5. Python·venv ──► 6. 시뮬레이터 패키지 (rosdep)
           ──► 7. 빌드·테스트 ──► 8. 시뮬레이터 확인 ① ──► 9. 웹캠 연결 ──► 10. 결과 기록
```

순서를 지켜야 하는 곳: **3 → 4 → 5** (conda 가 켜진 채로 venv 를 만들면 Python 3.11 venv 가 생김),
**4 → 6** (rosdep 은 ROS2 가 있어야 동작), **5·6 → 7** (`env.sh` 가 `.venv` 와 ROS2 를 같이 켬).
1·2 는 Windows 쪽이라 WSL 작업 전에 끝내 둡니다.

---

## 1. WSL 메모리 늘리기 (Windows)

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

## 2. usbipd 설치 (Windows, 웹캠 준비)

웹캠 연결은 9단계에서 하지만, 설치만 미리 해 둡니다.

```powershell
# PowerShell (관리자)
winget install usbipd
```

- [ ] `usbipd list` 가 USB 장치 목록을 보여 주면 통과

## 3. conda 자동 활성화 끄기

```bash
conda config --set auto_activate_base false
```

터미널을 **닫고 새로 연 뒤**:

```bash
which python3 && python3 --version
```

- [ ] `/usr/bin/python3`, `Python 3.12.x` 가 나오면 통과

> 다른 과제에서 conda 가 필요하면 그때 `conda activate <env>` 로 켜면 됩니다. 이 프로젝트의 `env.sh` 는 conda 가 켜져 있어도 먼저 끕니다.

## 4. ROS2 Jazzy 설치

SETUP.md Step 1 의 명령을 그대로 순서대로 실행합니다 (로케일 → apt 소스 → `ros-jazzy-desktop ros-dev-tools ros-jazzy-cv-bridge`).
다운로드 양이 많아서 시간이 꽤 걸립니다.

확인 (터미널 2개):

```bash
source /opt/ros/jazzy/setup.bash && ros2 run demo_nodes_cpp talker     # 터미널 A
source /opt/ros/jazzy/setup.bash && ros2 run demo_nodes_py listener    # 터미널 B
```

- [ ] 터미널 B 에 `I heard: [Hello World: N]` 가 계속 찍히면 통과

## 5. Python 패키지·venv

이 PC 에는 저장소가 이미 `~/study/robot_vision` 에 있으니 `git clone` 은 건너뜁니다 (다른 팀원은 SETUP.md Step 2 대로 clone).

```bash
sudo apt install -y python3-opencv python3-numpy python3-yaml python3-venv python3-pytest

cd ~/study/robot_vision
python3 -m venv --system-site-packages .venv
touch .venv/COLCON_IGNORE
source .venv/bin/activate
pip install -r requirements.txt          # PySide6, pytest 만 설치됨

source /opt/ros/jazzy/setup.bash
python -c "import rclpy, cv2, numpy, PySide6; print(cv2.__version__, numpy.__version__, PySide6.__version__)"
```

- [ ] 마지막 줄이 에러 없이 버전 3개를 출력하면 통과
- [ ] 출력된 버전을 10단계 표에 적어 둠

## 6. 시뮬레이터 패키지 (rosdep)

Gazebo Harmonic, UR5e, MoveIt2, pymoveit2 를 `src/ur5e_pick_place/package.xml` 기준으로 한 번에 설치합니다 (SETUP.md Step 3).

```bash
sudo apt install -y python3-rosdep
sudo rosdep init            # "already exists" 가 나오면 이미 된 것이라 넘어감
rosdep update

cd ~/study/robot_vision
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
```

- [ ] 마지막에 `#All required rosdeps installed successfully` 가 나오면 통과
- [ ] 못 찾는 키가 있으면 SETUP.md Step 3 표의 패키지를 `sudo apt install -y <이름>` 으로 직접 설치하고 10단계에 기록

## 7. 빌드·테스트

```bash
source ~/study/robot_vision/env.sh      # 이제부터는 매번 이 한 줄로 시작
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

- [ ] `colcon build` 가 `2 packages finished` 로 끝나면 통과
- [ ] 테스트는 실패(failure) 0 이면 통과. 뼈대라서 `skipped` 가 여러 개 나오는 것은 정상

## 8. 시뮬레이터 구동 확인 ① (Go / No-Go)

**8-1. UR 기본 예제** (우리 파일 없이):

```bash
source ~/study/robot_vision/env.sh
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

## 9. 웹캠 연결

```powershell
# PowerShell (관리자). 웹캠을 꽂은 상태에서
usbipd list                          # 웹캠 BUSID 확인 (예: 2-3)
usbipd bind --busid 2-3              # 처음 한 번만
usbipd attach --wsl --busid 2-3      # 웹캠을 다시 꽂거나 재부팅하면 다시
```

```bash
# WSL
sudo modprobe uvcvideo
ls /dev/video*
```

- [ ] `/dev/video0` 이 보이면 통과
- [ ] 끝내 안 되면 Windows 에서 촬영한 이미지 폴더를 `params.yaml` 의 `image_dir` 로 쓰기로 하고 팀에 알림

## 10. 결과 기록

설치가 끝나면 아래를 채웁니다. 팀원 3명의 값이 다르면 그 차이가 나중에 "내 PC 에서는 되는데" 문제의 원인이 됩니다.

| 기록할 것 | 어디에 |
|---|---|
| 8-2 런치 인자·xacro 경로 확인 결과, rosdep 이 못 찾은 키 | [DECISIONS.md 4장](DECISIONS.md#4-아직-확인-못-한-것-12주차에-채움) 표 |
| 확인 ① 통과 여부, 쓴 대체 방법 (소프트웨어 렌더링 등) | DECISIONS.md 4장 "WSL2 에서 Gazebo + MoveIt + RViz" 행 |
| 버전: PySide6·OpenCV·NumPy (5단계), `gz sim --version`, `apt-cache policy ros-jazzy-moveit` | PROJECT.md 7장 측정 장비 표 |
| PC 사양: CPU·RAM·GPU·WSL 메모리 설정 | PROJECT.md 7장 측정 장비 표 |

## 팀원별 진행 체크

| 단계 | 팀원 1 | 팀원 2 | 팀원 3 |
|---|---|---|---|
| 1. WSL 메모리 | | | |
| 2. usbipd | | | |
| 3. conda 끄기 | | | |
| 4. ROS2 Jazzy | | | |
| 5. Python·venv | | | |
| 6. rosdep | | | |
| 7. 빌드·테스트 | | | |
| 8. 확인 ① | | | |
| 9. 웹캠 | | | |
| 10. 기록 | | | |

환경 설치가 끝나면 1주차 나머지 일(체커보드 캘리브레이션, 격자 호모그래피, 로봇 좌표계 표시)로 넘어갑니다 (PROJECT.md 8장).

## 현재 설치 상태 (2026-11-10)

현재 단계는 **개발 환경 설치**입니다. 아래 결과는 이 PC 기준이며 팀원별로 확인해야 합니다.

| 항목 | 이 PC 상태 |
|---|---|
| WSL 메모리 | 9.7 GiB, swap 8.0 GiB |
| ROS2 | Jazzy 설치됨 |
| Python 환경 | venv, Python 3.12.3, OpenCV 4.6.0, NumPy 1.26.4, PySide6 6.12.0 |
| 시뮬레이터 패키지 | Gazebo Harmonic 8.15.0, MoveIt 2.12.4, UR MoveIt 설정 3.9.0 |
| ur_robot_driver | 시스템 미설치. 추가 설치 필요 |
| Windows usbipd | 미설치 |
| 웹캠 | Windows 카메라 장치 0개, WSL `/dev/video*` 없음 |

`ur_robot_driver`는 `rosdep install`로 설치하거나 다음 명령으로 직접 설치합니다.

```bash
sudo apt install ros-jazzy-ur-robot-driver
```

설치 확인: `source /opt/ros/jazzy/setup.bash` 후 `ros2 pkg prefix ur_robot_driver`를 실행합니다.
Windows usbipd 설치와 웹캠 연결은 위 2·9단계가 남아 있습니다.
