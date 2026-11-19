"""검출 로직 확인용 박스·중심·장축 미리보기. mm 보정·정답 평가는 수행하지 않는다.

PYTHONPATH=src/vision_pick_place python scripts/preview_detections.py \
    --images data/mvtec_screws/images --out logs/team2_detection_preview.jpg
"""
import argparse
from pathlib import Path

import cv2
import numpy as np

from vision_pick_place.camera_stream import FrameSource, preprocess
from vision_pick_place.vision_detector import classify_part, find_objects


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', required=True)
    parser.add_argument('--out', default='logs/team2_detection_preview.jpg')
    parser.add_argument('--count', type=int, default=6)
    parser.add_argument('--threshold', type=int, default=100)
    args = parser.parse_args()
    if not 1 <= args.count <= 24:
        parser.error('--count must be between 1 and 24')
    cv2.setNumThreads(1)
    source = FrameSource(image_dir=args.images)
    tiles = []
    try:
        for index in range(args.count):
            frame = source.read()
            if frame is None:
                raise RuntimeError('No readable image')
            # 이 도구는 gray·임계값 방식의 픽셀 검출 미리보기다. mm 보정이나 metal_hsv 평가는 하지 않는다.
            detections = find_objects(preprocess(frame, threshold=args.threshold))
            for det in detections:
                center = np.asarray(det.center_px)
                # 장축 단위벡터에 길이의 절반을 곱하면 중심에서 양 끝점까지의 벡터가 된다.
                axis = np.asarray(det.long_axis_px) * det.length_px / 2
                color = (0, 220, 0) if classify_part(det) == 'screw' else (0, 180, 255)
                # 박스는 종류별 색, 장축은 자홍색, 중심은 빨간 점으로 표시한다.
                cv2.polylines(frame, [np.rint(det.box_px).astype(np.int32)], True, color, 2)
                cv2.line(frame, tuple(np.rint(center-axis).astype(int)),
                         tuple(np.rint(center+axis).astype(int)), (255, 0, 255), 2)
                cv2.circle(frame, tuple(np.rint(center).astype(int)), 4, (0, 0, 255), -1)
            scale = min(640 / frame.shape[1], 420 / frame.shape[0])
            resized = cv2.resize(frame, None, fx=scale, fy=scale)
            tile = np.zeros((460, 640, 3), dtype=np.uint8)
            tile[:resized.shape[0], :resized.shape[1]] = resized
            cv2.putText(tile, f'Image {index+1}: {len(detections)} detections; threshold={args.threshold}',
                        (8, 445), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
            tiles.append(tile)
    finally:
        source.release()
    while len(tiles) % 2:
        tiles.append(np.zeros_like(tiles[0]))
    sheet = np.vstack([np.hstack(tiles[i:i+2]) for i in range(0, len(tiles), 2)])
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out), sheet):
        raise RuntimeError(f'Cannot save {out}')
    print(out)


if __name__ == '__main__':
    main()
