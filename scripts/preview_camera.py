"""카메라 없이 영상 입력·전처리 확인. 기본값은 MVTec 이미지 4장.

    python scripts/preview_camera.py
    python scripts/preview_camera.py --image-dir data/my_photos --count 10

결과는 data/preview 에 저장한다. 각 결과의 왼쪽은 원본, 오른쪽은 이진 마스크.
"""
import argparse
from pathlib import Path
import sys

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'vision_pick_place'))

from vision_pick_place.camera_stream import FrameSource, preprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image-dir', type=Path, default=ROOT / 'data/mvtec_screws/images')
    parser.add_argument('--out', type=Path, default=ROOT / 'data/preview')
    parser.add_argument('--count', type=int, default=4)
    parser.add_argument('--blur-ksize', type=int, default=5)
    parser.add_argument('--threshold', type=int, default=-1)
    args = parser.parse_args()
    if args.count < 1:
        parser.error('--count는 1 이상이어야 합니다.')
    try:
        source = FrameSource(image_dir=str(args.image_dir))
        try:
            args.out.mkdir(parents=True, exist_ok=True)
            for i in range(args.count):
                frame = source.read()
                if frame is None:
                    raise RuntimeError('읽을 수 있는 이미지가 없습니다.')
                mask = preprocess(frame, args.blur_ksize, args.threshold)
                comparison = cv2.hconcat([frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
                path = args.out / f'preview_{i + 1:03d}.png'
                if not cv2.imwrite(str(path), comparison):
                    raise RuntimeError(f'결과를 저장할 수 없습니다: {path}')
                print(f'{path} (왼쪽: 원본 / 오른쪽: 이진화)')
        finally:
            source.release()
    except (OSError, ValueError, RuntimeError, cv2.error) as exc:
        parser.exit(1, f'오류: {exc}\n')


if __name__ == '__main__':
    main()
