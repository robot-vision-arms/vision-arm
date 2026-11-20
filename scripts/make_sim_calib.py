"""Gazebo 가상 카메라용 작업 시트 그림·보정 파일 (DECISIONS.md 12절).

실물과 같은 보정 과정을 시뮬레이터에도 쓴다: Gazebo 작업 매트(work_mat)에 실제 작업 시트와 같은 위치·크기의
ArUco 마커(ID 40~43)를 그려 두고, 가상 카메라 사진으로 compute_homography.py --aruco 를 그대로 돌린다.

    # 1) 매트 그림 + 내부 파라미터 (Gazebo 를 켜기 전, 카메라 설정을 바꿨을 때만)
    PYTHONPATH=src/vision_pick_place python scripts/make_sim_calib.py
    # 2) 시뮬레이터를 띄운 뒤 (ros2 launch ur5e_pick_place sim.launch.py) 가상 카메라 한 장
    python scripts/make_sim_calib.py --capture data/calib/sim/sheet.png
    # 3) 실물과 같은 명령으로 호모그래피
    PYTHONPATH=src/vision_pick_place python scripts/compute_homography.py --aruco \\
        --image data/calib/sim/sheet.png --intrinsics data/calib/sim/intrinsics.npz --out data/calib/sim/homography.npz

매트 그림 규칙:
    - 바탕은 workcell.sdf 의 초록 (0.10, 0.60, 0.20). 회색 금속 부품과 채도가 달라 metal_hsv 로 분리된다
    - 마커 둘레 흰 여백 8 mm (실물 시트는 4 mm). 초록 바탕에서는 흰 여백 바깥 테두리도 사각형으로 보이는데,
      ArUco 는 둘레의 5 % (35 mm 마커면 약 7 mm) 보다 가까운 두 사각형을 하나로 보고 바깥 것만 남겨 마커를 놓친다
      (4 mm 로는 0개 검출, 11-20 확인). 흰 바탕인 실물 시트는 바깥 테두리가 없어 4 mm 로 된다.
    - 그래서 가상 카메라에서는 vision_node 의 sheet_marker_margin_mm 를 9 로 준다. 흰 여백이 검출 영역에 남으면
      흰색은 채도가 낮아 metal_hsv 에서 부품으로 잡힌다
내부 파라미터: 가상 카메라는 왜곡이 없고, 초점거리는 가로 시야각으로 정해진다. fx = fy = (W/2) / tan(hfov/2), 중심 = 화면 가운데.
"""
import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / 'src/ur5e_pick_place/worlds/workcell.sdf'
TEXTURE = ROOT / 'src/ur5e_pick_place/models/work_mat/work_sheet.png'
OUT_DIR = ROOT / 'data/calib/sim'
GREEN_RGB = (0.10, 0.60, 0.20)   # workcell.sdf 의 초록과 같은 값
PX_PER_MM = 8.0                  # 그림 해상도. 가상 카메라는 1 mm ≈ 3 px 라 충분하다
QUIET_MM = 8.0                   # 마커 둘레 흰 여백 (위 설명. 검출 영역은 마커 둘레 9 mm 를 빼야 한다)


def render_mat(px_per_mm: float = PX_PER_MM) -> np.ndarray:
    """작업 시트(A4 가로) 크기 BGR 그림. 행 0 = 시트 위쪽(y = 210 mm), 열 0 = 왼쪽(x = 0)."""
    from vision_pick_place import aruco_calib as ac

    width, height = (round(v * px_per_mm) for v in ac.SHEET_MM)
    image = np.empty((height, width, 3), np.uint8)
    image[:] = [round(255 * c) for c in reversed(GREEN_RGB)]
    side, quiet = round(ac.SHEET_MARKER_MM * px_per_mm), round(QUIET_MM * px_per_mm)
    for marker_id, (x, y) in ac.SHEET_MARKERS.items():
        col, row = round(x * px_per_mm), round((ac.SHEET_MM[1] - y - ac.SHEET_MARKER_MM) * px_per_mm)
        image[row - quiet:row + side + quiet, col - quiet:col + side + quiet] = 255
        image[row:row + side, col:col + side] = cv2.cvtColor(ac.marker_image(marker_id, side), cv2.COLOR_GRAY2BGR)
    return image


def camera_intrinsics(world: Path = WORLD):
    """workcell.sdf 의 top_camera 설정 → (camera_matrix, (너비, 높이))."""
    sensor = ET.parse(world).getroot().find(".//model[@name='top_camera']//sensor[@type='camera']/camera")
    if sensor is None:
        raise ValueError(f'top_camera 카메라 센서가 없습니다: {world}')
    hfov = float(sensor.findtext('horizontal_fov'))
    width, height = int(sensor.findtext('image/width')), int(sensor.findtext('image/height'))
    f = (width / 2) / math.tan(hfov / 2)
    return np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1]], dtype=np.float64), (width, height)


def capture(path: Path, topic: str = '/sim/camera/image', timeout_s: float = 15.0) -> None:
    """가상 카메라 한 장 저장. 렌더링이 안정되도록 세 번째 프레임을 쓴다."""
    import rclpy
    from sensor_msgs.msg import Image

    rclpy.init()
    node, frames = rclpy.create_node('sim_calib_capture'), []
    node.create_subscription(Image, topic, frames.append, 5)
    deadline = node.get_clock().now().nanoseconds + timeout_s * 1e9
    while len(frames) < 3 and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()
    if len(frames) < 3:
        raise RuntimeError(f'{topic} 에서 영상을 받지 못했습니다. sim.launch.py 가 떠 있는지 확인하세요.')
    m = frames[-1]
    image = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    if m.encoding == 'rgb8':
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), image)
    print('saved', path)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--capture', type=Path, help='시뮬레이터가 떠 있을 때 가상 카메라 한 장을 이 경로에 저장')
    args = parser.parse_args()
    if args.capture:
        capture(args.capture)
        return
    TEXTURE.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(TEXTURE), render_mat())
    print('texture', TEXTURE)
    K, size = camera_intrinsics()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / 'intrinsics.npz'
    np.savez(out, camera_matrix=K, dist_coeffs=np.zeros(5), rms=0.0, image_size=np.array(size))
    print('intrinsics', out, f'fx = fy = {K[0, 0]:.1f}, size {size}')


if __name__ == '__main__':
    main()
