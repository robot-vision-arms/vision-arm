#!/usr/bin/env python3
"""설치 상태 조회 전용. 추가 Python 패키지 없이 실행한다.

    source ./env.sh
    python scripts/check_environment.py
    python scripts/check_environment.py --json

설치·빌드·카메라 촬영·로봇 이동은 수행하지 않는다.
PASS=확인 성공, FAIL=필수 조건 미충족, WARN=추가 확인 필요,
MANUAL=직접 실행해야 하는 검증. 종료 코드는 FAIL이 있으면 1, 없으면 0이다.
종료 코드 0도 수동 구동 검증 완료를 의미하지 않는다.
"""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
RESULTS = []


def record(status, name, detail, action=''):
    RESULTS.append(dict(status=status, name=name, detail=str(detail), action=action))


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
    record('PASS' if status == 0 else 'FAIL', name, output[-1200:], action if status != 0 else '')


def system_checks():
    try:
        release = platform.freedesktop_os_release()
    except OSError:
        release = {}
    ubuntu = release.get('ID') == 'ubuntu' and release.get('VERSION_ID') == '24.04'
    record('PASS' if ubuntu else 'FAIL', 'Ubuntu 24.04',
           release.get('PRETTY_NAME', platform.platform()), 'SETUP.md 환경 사양 확인' if not ubuntu else '')
    is_wsl = 'microsoft' in platform.release().lower()
    record('PASS' if is_wsl else 'INFO', 'WSL', platform.release())
    record('INFO', 'CPU', f'{os.cpu_count()} logical CPUs / {platform.machine()}')
    memory = Path('/proc/meminfo')
    if memory.exists():
        values = dict(re.findall(r'^(MemTotal|SwapTotal):\s+(\d+)', memory.read_text(), re.M))
        ram = int(values.get('MemTotal', 0)) / 1024**2
        swap = int(values.get('SwapTotal', 0)) / 1024**2
        # WSL의 8GB 설정도 커널 예약분 때문에 8GiB 미만으로 표시된다.
        record('PASS' if ram >= 7.5 else 'WARN', 'WSL RAM / swap',
               f'{ram:.1f} GiB / {swap:.1f} GiB',
               'WSL 메모리 8GB 이상 권장 (WEEK1.md 1단계)' if ram < 7.5 else '')
    else:
        record('WARN', 'RAM', '/proc/meminfo 조회 불가')
    free = shutil.disk_usage(ROOT).free / 1024**3
    record('PASS' if free >= 15 else 'WARN', '디스크 여유', f'{free:.1f} GiB',
           '설치 여유 공간 15GB 이상 권장' if free < 15 else '')
    correct_python = sys.version_info[:2] == (3, 12)
    record('PASS' if correct_python else 'FAIL', 'Python 3.12',
           f'{platform.python_version()} / {sys.executable}',
           '시스템 Python 3.12로 .venv 생성' if not correct_python else '')
    in_project_venv = Path(sys.prefix).absolute() == (ROOT / '.venv').absolute()
    record('PASS' if in_project_venv else 'WARN', '프로젝트 venv 활성화', sys.prefix,
           'source ./env.sh' if not in_project_venv else '')
    config = ROOT / '.venv/pyvenv.cfg'
    shared = config.exists() and bool(re.search(r'include-system-site-packages\s*=\s*true', config.read_text(), re.I))
    record('PASS' if shared else 'FAIL', 'venv 시스템 패키지 공유', str(config),
           'python3 -m venv --system-site-packages .venv' if not shared else '')
    ignored = (ROOT / '.venv/COLCON_IGNORE').is_file()
    record('PASS' if ignored else 'WARN', 'venv COLCON_IGNORE', ignored,
           'touch .venv/COLCON_IGNORE' if not ignored else '')
    active_conda = os.environ.get('CONDA_PREFIX')
    record('FAIL' if active_conda else 'PASS', 'conda 비활성화', active_conda or '비활성화',
           'conda deactivate 후 source ./env.sh' if active_conda else '')
    record('MANUAL', 'conda 자동 활성화 설정', '현재 세션의 비활성화와 별도로 확인',
           '새 터미널에서 which python3 및 conda config --show auto_activate_base 확인')
    return is_wsl


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
    record('PASS' if distro == 'jazzy' else 'FAIL', 'ROS_DISTRO', distro or '미설정',
           'source ./env.sh' if distro != 'jazzy' else '')
    record('PASS' if Path('/opt/ros/jazzy/setup.bash').is_file() else 'FAIL',
           'ROS2 설치 파일', '/opt/ros/jazzy/setup.bash')
    for command in ('git', 'ros2', 'colcon', 'rosdep', 'gz', 'xacro'):
        executable = shutil.which(command)
        record('PASS' if executable else 'FAIL', '실행 파일: ' + command, executable or 'PATH에 없음',
               'SETUP.md 설치 및 source ./env.sh 확인' if not executable else '')
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
            record('FAIL', 'package.xml 읽기', f'{manifest}: {exc}')
    if not projects:
        record('FAIL', '프로젝트 패키지', 'src/*/package.xml 없음')
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
        record('FAIL', 'ROS 패키지 인덱스', output[-1000:], 'source ./env.sh 및 ament_index_python 설치 확인')
    else:
        for package in sorted(dependencies | projects):
            prefix = prefixes.get(package)
            detail = prefix or '인덱스에 없음'
            if prefix:
                try:
                    version = ET.parse(Path(prefix) / 'share' / package / 'package.xml').getroot().findtext('version')
                    detail = f'{prefix} / version={version}'
                except (OSError, ET.ParseError):
                    record('WARN', 'ROS 패키지 메타데이터: ' + package, 'package.xml 버전 조회 불가')
            record('PASS' if prefix else 'FAIL', 'ROS 패키지: ' + package, detail,
                   ('colcon build --symlink-install 후 source install/setup.bash' if package in projects
                    else 'rosdep install --from-paths src --ignore-src -y --rosdistro jazzy') if not prefix else '')
    for package in sorted(apt_packages):
        rc, output = run(['dpkg-query', '-W', '-f=${Status} ${Version}', package])
        installed = rc == 0 and output.startswith('install ok installed ')
        record('PASS' if installed else 'FAIL', 'apt 패키지: ' + package,
               output if installed else '설치 확인 실패', 'sudo apt install ' + package if not installed else '')
    if shutil.which('gz'):
        rc, output = run(['gz', 'sim', '--version'])
        harmonic = rc == 0 and bool(re.search(r'version\s+8\.', output, re.I))
        record('PASS' if harmonic else 'FAIL', 'Gazebo Harmonic', output[:500],
               'ros-jazzy-ur-simulation-gz 및 ros_gz_sim 설치 확인' if not harmonic else '')
    rosdep_config = Path('/etc/ros/rosdep/sources.list.d/20-default.list')
    record('PASS' if rosdep_config.is_file() else 'WARN', 'rosdep 초기화', str(rosdep_config),
           'sudo rosdep init 후 rosdep update' if not rosdep_config.is_file() else '')
    record('MANUAL', 'rosdep 의존성 전체 점검', '규칙 해석과 캐시 최신 여부는 별도 확인',
           'rosdep check --from-paths src --ignore-src --rosdistro jazzy')
    built = (ROOT / 'install/setup.bash').is_file()
    record('PASS' if built else 'WARN', '빌드 설치 파일 존재', built,
           'colcon build --symlink-install' if not built else '')
    record('MANUAL', '현재 소스 빌드·테스트', '파일 존재만으로 최신 빌드 성공은 판정하지 않음',
           'colcon build --symlink-install && colcon test && colcon test-result --verbose')


def devices(is_wsl, skip_windows):
    display = os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')
    record('PASS' if display else 'WARN', '화면 세션 환경 변수', display or '없음',
           'WSLg 또는 화면 세션 확인' if not display else '')
    record('INFO', '소프트웨어 렌더링 설정', os.environ.get('LIBGL_ALWAYS_SOFTWARE', '미설정'))
    video = sorted(Path('/dev').glob('video*'))
    record('PASS' if video else 'WARN', 'WSL/Linux 웹캠 장치',
           ', '.join(str(p) for p in video) or '장치 없음',
           'WEEK1.md 2·9단계: usbipd 설치·웹캠 연결. 이미지 폴더로 대체 가능' if not video else '')
    if video:
        inaccessible = [str(p) for p in video if not os.access(p, os.R_OK | os.W_OK)]
        record('WARN' if inaccessible else 'PASS', '웹캠 장치 접근 권한',
               ', '.join(inaccessible) if inaccessible else '읽기·쓰기 권한 있음')
    if is_wsl and not skip_windows:
        powershell = shutil.which('powershell.exe')
        if powershell:
            code = ("$u=Get-Command usbipd -ErrorAction SilentlyContinue; "
                    "$c=@(Get-PnpDevice -PresentOnly -ErrorAction Stop | "
                    "Where-Object {$_.Class -in @('Camera','Image')}); "
                    "[pscustomobject]@{usbipd=[bool]$u; cameras=$c.Count; "
                    "gpu=@(Get-CimInstance Win32_VideoController -ErrorAction Stop | "
                    "Select-Object -ExpandProperty Name)} | ConvertTo-Json -Compress")
            rc, output = run([powershell, '-NoProfile', '-NonInteractive', '-Command', code], timeout=20)
            try:
                data = json.loads(output) if rc == 0 else None
            except ValueError:
                data = None
            if data is None:
                record('WARN', 'Windows 조회', '조회 불가: ' + output[-500:],
                       'Windows PowerShell에서 usbipd list 및 카메라 장치 직접 확인')
            else:
                record('PASS' if data['usbipd'] else 'WARN', 'Windows usbipd',
                       '설치됨' if data['usbipd'] else '명령을 찾을 수 없음',
                       '관리자 PowerShell: winget install usbipd' if not data['usbipd'] else '')
                record('PASS' if data['cameras'] else 'WARN', 'Windows 카메라',
                       f"{data['cameras']}개 (WSL 연결·영상 수신 성공 여부는 별도)")
                record('INFO', 'Windows GPU', ', '.join(data['gpu']))
        else:
            record('WARN', 'Windows 조회', 'powershell.exe가 PATH에 없음', 'Windows에서 직접 확인')
    elif is_wsl:
        record('WARN', 'Windows 조회', '--skip-windows로 생략')
    record('MANUAL', '실제 영상 수신', '장치 존재만 점검. 카메라는 열지 않음',
           '사용할 웹캠 영상 또는 대체 이미지 폴더 입력 확인')
    record('MANUAL', 'ROS2 노드 통신', 'ROS daemon·노드는 실행하지 않음',
           'SETUP.md Step 1: 별도 터미널의 talker/listener 송수신 확인')
    record('MANUAL', 'Gazebo·RViz·MoveIt 구동', 'GUI 생성·계획·실행 성공은 설치 조회로 판정 불가',
           'SETUP.md Step 5: UR 기본 예제 실행 후 RViz Plan & Execute 확인')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--json', action='store_true', help='JSON 결과만 출력')
    parser.add_argument('--skip-windows', action='store_true', help='Windows PowerShell 조회 생략')
    args = parser.parse_args()
    if not args.json:
        print(f'환경 설치 점검: {ROOT}\n각 외부 조회에는 제한 시간이 있습니다. 잠시 기다려 주세요.', flush=True)
    is_wsl = system_checks()
    python_checks()
    ros_checks()
    devices(is_wsl, args.skip_windows)
    counts = dict(Counter(item['status'] for item in RESULTS))
    if args.json:
        print(json.dumps(dict(workspace=str(ROOT), summary=counts, checks=RESULTS), ensure_ascii=False, indent=2))
    else:
        for item in RESULTS:
            print(f"[{item['status']}] {item['name']}: {item['detail']}")
            if item['action']:
                print(f"       확인/조치: {item['action']}")
        print('\n결과: ' + ', '.join(f'{key} {value}' for key, value in counts.items()))
        print('MANUAL 항목은 직접 확인해야 합니다. 설치 조회 성공만으로 환경 전체 완료를 뜻하지 않습니다.')
    return 1 if counts.get('FAIL', 0) else 0


if __name__ == '__main__':
    sys.exit(main())
