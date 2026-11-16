"""단계별 처리 영상과 코드를 비교한다. ROS·카메라 없이 실행 가능.

python scripts/preview_steps.py
python scripts/preview_steps.py --image data/captures/part.png --mode gray
python scripts/preview_steps.py --export-only --out data/pipeline_steps
"""
import argparse
from pathlib import Path
import sys

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/vision_pick_place'))

from vision_pick_place.inspection import export_steps, inspect_frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path)
    parser.add_argument('--image-dir', type=Path, default=ROOT / 'data/mvtec_screws/images')
    parser.add_argument('--mode', choices=['gray', 'metal_hsv'], default='metal_hsv')
    parser.add_argument('--export-only', action='store_true', help='창 없이 첫 이미지의 단계 결과 저장')
    parser.add_argument('--out', type=Path, default=ROOT / 'data/pipeline_steps')
    args = parser.parse_args()
    cv2.setNumThreads(1)
    if args.image:
        paths = [args.image]
    else:
        paths = sorted(path for path in args.image_dir.glob('*')
                       if path.suffix.lower() in ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff'))
    if not paths or not paths[0].is_file():
        parser.error('이미지가 없습니다. --image 또는 --image-dir로 사진 경로를 지정하세요.')
    if args.export_only:
        frame = cv2.imread(str(paths[0]))
        if frame is None:
            parser.error(f'이미지를 읽지 못했습니다: {paths[0]}')
        steps = inspect_frame(frame, segmentation_mode=args.mode, blur_ksize=5,
                              threshold=-1, hsv_max_saturation=70, hsv_max_dark_value=90,
                              close_ksize=3 if args.mode == 'metal_hsv' else 0,
                              open_ksize=3 if args.mode == 'metal_hsv' else 0)
        export_steps(steps, args.out / paths[0].stem)
        print(f'{len(steps)}개 단계 저장: {args.out / paths[0].stem}')
        return
    from PySide6.QtWidgets import QApplication
    from vision_pick_place.gui.pipeline_inspector import PipelineInspector

    app = QApplication(sys.argv[:1])
    window = PipelineInspector(paths, mode=args.mode, export_dir=args.out)
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
