#!/usr/bin/env python3
"""설치 상태 조회 전용. 추가 Python 패키지 없이 실행한다.

    source ./env.sh
    python scripts/check_environment.py
    python scripts/check_environment.py --json

설치·빌드·영상 수신·로봇 이동은 수행하지 않는다. 스마트폰 카메라는 접속만 확인하고 영상은 열지 않는다.
OK=확인 성공, FAIL=조건 미충족(조치 필요). 종료 코드는 FAIL이 있으면 1, 없으면 0이다.
조회로 판정할 수 없는 구동 검증은 마지막 "직접 확인할 항목"에 따로 적는다.
종료 코드 0도 그 항목의 완료를 의미하지 않는다.
"""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import platform
import re
import shutil
import socket
import subprocess
import sys
from urllib.parse import urlparse
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PARAMS = ROOT / 'src/vision_pick_place/config/params.yaml'
RESULTS = []
MANUAL = []


def record(ok, name, detail, action=''):
    RESULTS.append(dict(status='OK' if ok else 'FAIL', name=name, detail=str(detail),
                        action='' if ok else action))


def manual(name, action):
    MANUAL.append(dict(name=name, action=action))


def run(command, timeout=12):
    try:
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True,
                                errors='replace', timeout=timeout)
        output = (result.stdout + result.stderr).strip()
        return result.returncode, output
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, str(exc)


def probe(name, code, action):
    status, output = run([sys.executable, '-c', code])
    record(status == 0, name, output[-1200:], action)


def system_checks():
    try:
        release = platform.freedesktop_os_release()
    except OSError:
        release = {}
    ubuntu = release.get('ID') == 'ubuntu' and release.get('VERSION_ID') == '24.04'
    record(ubuntu, 'Ubuntu 24.04', release.get('PRETTY_NAME', platform.platform()), 'SETUP.md 환경 사양 확인')
    is_wsl = 'microsoft' in platform.release().lower()
    record(True, 'WSL', platform.release() if is_wsl else 'WSL 아님: ' + platform.release())
    record(True, 'CPU', f'{os.cpu_count()} logical CPUs / {platform.machine()}')
    memory = Path('/proc/meminfo')
    values = dict(re.findall(r'^(MemTotal|SwapTotal):\s+(\d+)', memory.read_text(), re.M)) if memory.exists() else {}
    ram = int(values.get('MemTotal', 0)) / 1024**2
    swap = int(values.get('SwapTotal', 0)) / 1024**2
    # WSL의 8GB 설정도 커널 예약분 때문에 8GiB 미만으로 표시된다.
    record(ram >= 7.5, 'WSL RAM / swap', f'{ram:.1f} GiB / {swap:.1f} GiB',
           'WSL 메모리 8GB 이상 (WEEK1.md 1단계)')
    free = shutil.disk_usage(ROOT).free / 1024**3
    record(free >= 15, '디스크 여유', f'{free:.1f} GiB', '설치 여유 공간 15GB 이상 확보')
    record(sys.version_info[:2] == (3, 12), 'Python 3.12', f'{platform.python_version()} / {sys.executable}',
           '시스템 Python 3.12로 .venv 생성')
    in_project_venv = Path(sys.prefix).absolute() == (ROOT / '.venv').absolute()
    record(in_project_venv, '프로젝트 venv 활성화', sys.prefix, 'source ./env.sh')
    config = ROOT / '.venv/pyvenv.cfg'
    shared = config.exists() and bool(re.search(r'include-system-site-packages\s*=\s*true', config.read_text(), re.I))
    record(shared, 'venv 시스템 패키지 공유', str(config), 'python3 -m venv --system-site-packages .venv')
    ignored = (ROOT / '.venv/COLCON_IGNORE').is_file()
    record(ignored, 'venv COLCON_IGNORE', ignored, 'touch .venv/COLCON_IGNORE')
    active_conda = os.environ.get('CONDA_PREFIX')
    record(not active_conda, 'conda 비활성화', active_conda or '비활성화', 'conda deactivate 후 source ./env.sh')
    manual('conda 자동 활성화 꺼짐', '새 터미널에서 which python3 및 conda config --show auto_activate_base 확인')


def python_checks():
    actions = {
        'rclpy': 'ROS2 Jazzy 설치 후 source ./env.sh',
        'cv2': 'sudo apt install python3-opencv',
        'numpy': 'sudo apt install python3-numpy (프로젝트는 NumPy 1.x 사용)',
        'yaml': 'sudo apt install python3-yaml',
        'PySide6': 'venv에서 pip install -r requirements.txt',
        'pytest': 'sudo apt install python3-pytest',
        'pymoveit2': 'sudo apt install ros-jazzy-pymoveit2 후 source ./env.sh',
    }
    for module, action in actions.items():
        code = (f'import {module} as m; '
                "print('version=' + str(getattr(m, '__version__', '버전 속성 없음'))); "
                "print('path=' + str(getattr(m, '__file__', '없음')))\n")
        if module == 'numpy':
            code += "assert int(m.__version__.split('.')[0]) < 2, 'ROS cv_bridge와 NumPy 2.x 충돌 가능'\n"
        if module == 'PySide6':
            code += "assert (6, 6) <= tuple(map(int, m.__version__.split('.')[:2])) < (7, 0), 'requirements.txt 버전 범위 불일치'\n"
        probe('Python import: ' + module, code, action)
    probe('cv_bridge ↔ NumPy 변환',
          "import cv2, numpy as np; from cv_bridge import CvBridge; "
          "b=CvBridge(); a=np.zeros((8,8,3), dtype=np.uint8); "
          "r=b.imgmsg_to_cv2(b.cv2_to_imgmsg(a, encoding='bgr8'), desired_encoding='bgr8'); "
          "assert np.array_equal(a,r); print('BGR 이미지 왕복 변환 성공')",
          'sudo apt install ros-jazzy-cv-bridge python3-numpy python3-opencv 후 source ./env.sh')
    probe('Qt offscreen 초기화',
          "import os; os.environ['QT_QPA_PLATFORM']='offscreen'; "
          "from PySide6.QtWidgets import QApplication, QLabel; "
          "app=QApplication([]); label=QLabel('check'); "
          "app.processEvents(); print('Qt 초기화 성공 (실제 화면 표시는 별도 확인)')",
          'PySide6 및 Qt 런타임 의존성 확인')


def ros_checks():
    distro = os.environ.get('ROS_DISTRO')
    record(distro == 'jazzy', 'ROS_DISTRO', distro or '미설정', 'source ./env.sh')
    record(Path('/opt/ros/jazzy/setup.bash').is_file(), 'ROS2 설치 파일', '/opt/ros/jazzy/setup.bash',
           'SETUP.md Step 1: ROS2 Jazzy 설치')
    for command in ('git', 'ros2', 'colcon', 'rosdep', 'gz', 'xacro'):
        executable = shutil.which(command)
        record(bool(executable), '실행 파일: ' + command, executable or 'PATH에 없음',
               'SETUP.md 설치 및 source ./env.sh 확인')
    dependencies = {'rviz2', 'moveit_ros_move_group', 'controller_manager', 'gz_ros2_control'}
    apt_packages = set()
    projects = set()
    for manifest in sorted((ROOT / 'src').glob('*/package.xml')):
        try:
            root = ET.parse(manifest).getroot()
            projects.add(root.findtext('name'))
            for tag in ('exec_depend', 'depend', 'test_depend'):
                for entry in root.findall(tag):
                    name = (entry.text or '').strip()
                    if name.startswith('python3-'):
                        apt_packages.add(name)
                    elif name:
                        dependencies.add(name)
        except (ET.ParseError, OSError) as exc:
            record(False, 'package.xml 읽기', f'{manifest}: {exc}', 'package.xml 문법 확인')
    if not projects:
        record(False, '프로젝트 패키지', 'src/*/package.xml 없음', '저장소 src/ 확인')
    # shell이나 ROS daemon 없이 ament 인덱스를 한 번 조회한다.
    rc, output = run([sys.executable, '-c',
                      "import json; from ament_index_python.packages import get_packages_with_prefixes; "
                      "p=get_packages_with_prefixes(); print(json.dumps({k:p.get(k) for k in "
                      + repr(sorted(dependencies | projects)) + "}))"])
    try:
        prefixes = json.loads(output) if rc == 0 else None
    except ValueError:
        prefixes = None
    if prefixes is None:
        record(False, 'ROS 패키지 인덱스', output[-1000:], 'source ./env.sh 및 ament_index_python 설치 확인')
    else:
        for package in sorted(dependencies | projects):
            prefix = prefixes.get(package)
            detail = prefix or '인덱스에 없음'
            if prefix:
                try:
                    version = ET.parse(Path(prefix) / 'share' / package / 'package.xml').getroot().findtext('version')
                    detail = f'{prefix} / version={version}'
                except (OSError, ET.ParseError):
                    detail = f'{prefix} / 버전 조회 불가'
            record(bool(prefix), 'ROS 패키지: ' + package, detail,
                   'colcon build --symlink-install 후 source install/setup.bash' if package in projects
                   else 'rosdep install --from-paths src --ignore-src -y --rosdistro jazzy')
    for package in sorted(apt_packages):
        rc, output = run(['dpkg-query', '-W', '-f=${Status} ${Version}', package])
        installed = rc == 0 and output.startswith('install ok installed ')
        record(installed, 'apt 패키지: ' + package, output if installed else '설치 확인 실패',
               'sudo apt install ' + package)
    if shutil.which('gz'):
        rc, output = run(['gz', 'sim', '--version'])
        harmonic = rc == 0 and bool(re.search(r'version\s+8\.', output, re.I))
        record(harmonic, 'Gazebo Harmonic', output[:500], 'ros-jazzy-ur-simulation-gz 및 ros_gz_sim 설치 확인')
    rosdep_config = Path('/etc/ros/rosdep/sources.list.d/20-default.list')
    record(rosdep_config.is_file(), 'rosdep 초기화', str(rosdep_config), 'sudo rosdep init 후 rosdep update')
    manual('rosdep 의존성 전체 점검', 'rosdep check --from-paths src --ignore-src --rosdistro jazzy')
    built = (ROOT / 'install/setup.bash').is_file()
    record(built, '빌드 설치 파일 존재', built, 'colcon build --symlink-install')
    manual('현재 소스 빌드·테스트', 'colcon build --symlink-install && colcon test && colcon test-result --verbose')


def load_vision_params():
    try:
        import yaml
        return yaml.safe_load(PARAMS.read_text())['vision_node']['ros__parameters']
    except Exception as exc:  # 설정 파일이 깨졌거나 yaml이 없을 때 모두 같은 조치
        return exc


def camera_checks():
    display = os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')
    record(bool(display), '화면 세션 환경 변수', display or '없음', 'WSLg 또는 화면 세션 확인')
    record(True, '소프트웨어 렌더링 설정', os.environ.get('LIBGL_ALWAYS_SOFTWARE', '미설정'))
    params = load_vision_params()
    if isinstance(params, Exception):
        record(False, '비전 입력 설정', f'{PARAMS}: {params}', 'params.yaml 의 vision_node 항목 확인')
        return
    image_dir = params.get('image_dir') or ''
    camera_url = params.get('camera_url') or ''
    if image_dir:
        # 대체 입력(사진 폴더·동영상 파일)을 쓰는 중이면 스마트폰 접속은 점검하지 않는다.
        path = ROOT / image_dir
        record(path.exists(), '비전 입력: image_dir', path, 'params.yaml 의 image_dir 경로 확인')
        return
    url = urlparse(camera_url)
    if url.scheme not in ('http', 'https', 'rtsp') or not url.hostname:
        record(False, '스마트폰 카메라 주소', camera_url or 'camera_url 미설정',
               'SETUP.md Step 7: 폰 앱에 표시된 주소를 params.yaml 의 camera_url 에 입력')
        return
    record(True, '스마트폰 카메라 주소', camera_url)
    port = url.port or {'http': 80, 'https': 443, 'rtsp': 554}[url.scheme]
    try:
        # 영상은 열지 않고 TCP 접속만 확인한다.
        with socket.create_connection((url.hostname, port), timeout=3):
            pass
        record(True, '스마트폰 카메라 접속', f'{url.hostname}:{port} 접속됨')
    except OSError as exc:
        record(False, '스마트폰 카메라 접속', f'{url.hostname}:{port} 접속 불가 ({exc})',
               '폰 앱 서버 실행, PC와 같은 Wi-Fi, 폰 IP 변경 여부 확인 (SETUP.md Step 7)')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--json', action='store_true', help='JSON 결과만 출력')
    args = parser.parse_args()
    if not args.json:
        print(f'환경 설치 점검: {ROOT}\n각 외부 조회에는 제한 시간이 있습니다. 잠시 기다려 주세요.', flush=True)
    system_checks()
    python_checks()
    ros_checks()
    camera_checks()
    manual('스마트폰 영상 수신', 'SETUP.md Step 7 의 확인 명령으로 프레임 1장 읽기')
    manual('ROS2 노드 통신', 'SETUP.md Step 1: 별도 터미널의 talker/listener 송수신 확인')
    manual('Gazebo·RViz·MoveIt 구동', 'SETUP.md Step 5: UR 기본 예제 실행 후 RViz Plan & Execute 확인')
    counts = Counter(item['status'] for item in RESULTS)
    if args.json:
        print(json.dumps(dict(workspace=str(ROOT), summary=dict(counts), checks=RESULTS, manual=MANUAL),
                         ensure_ascii=False, indent=2))
    else:
        for item in RESULTS:
            print(f"[{item['status']:<4}] {item['name']}: {item['detail']}")
            if item['action']:
                print(f"       조치: {item['action']}")
        print('\n직접 확인할 항목 (조회로 판정 불가):')
        for item in MANUAL:
            print(f"  [ ] {item['name']}: {item['action']}")
        print(f"\n결과: OK {counts['OK']}, FAIL {counts['FAIL']}")
    return 1 if counts['FAIL'] else 0


if __name__ == '__main__':
    sys.exit(main())
