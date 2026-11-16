"""실제 전처리의 중간 결과와 코드 설명. GUI 없이도 단계 이미지를 저장할 수 있다."""
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from vision_pick_place.camera_stream import preprocess
from vision_pick_place.vision_detector import classify_part, find_objects


@dataclass
class ProcessingStep:
    key: str
    title: str
    description: str
    code: str
    parent: str
    image: np.ndarray


def inspect_frame(frame, *, min_area_px=500, nut_max_aspect=1.5, **options):
    captured = {}
    mask = preprocess(frame, on_step=lambda key, image: captured.update({key: image.copy()}), **options)
    gray_mode = options.get('segmentation_mode', 'gray') == 'gray'
    blur = options.get('blur_ksize', 5)
    sat, dark = options.get('hsv_max_saturation', 70), options.get('hsv_max_dark_value', 70)
    threshold = options.get('threshold', -1)
    descriptions = {
        'original': ('원본', '파일에서 읽은 BGR 영상. 아래 단계도 이 해상도 그대로 처리합니다.',
                     'frame = cv2.imread(path)', 'original'),
        'grayscale': ('회색조', '색 정보를 제거하고 픽셀의 밝기만 남깁니다.',
                      'gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)', 'original'),
        'blur': ('Gaussian Blur', f'{blur}×{blur} 범위로 작은 밝기 변화를 부드럽게 합니다. 1이면 변화가 없습니다.',
                 f'blurred = cv2.GaussianBlur(image, ({blur}, {blur}), 0)',
                 'grayscale' if gray_mode else 'original'),
        'hsv_saturation': ('채도 S', 'Blur 영상에서 분리한 채도입니다. 어두울수록 무채색에 가깝습니다. V와는 별도 채널입니다.',
                           'hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)\nS = hsv[:, :, 1]', 'blur'),
        'hsv_value': ('밝기 V', 'Blur 영상에서 분리한 밝기입니다. 어두울수록 V가 작습니다. S와는 별도 채널입니다.',
                      'V = hsv[:, :, 2]', 'blur'),
        'threshold': ('이진화',
                      ('밝기가 임계값 이하인 픽셀을 흰색 물체 후보로 남깁니다. -1은 Otsu 자동 임계값입니다.' if gray_mode
                       else f'S ≤ {sat} 또는 V ≤ {dark}인 픽셀을 흰색으로 선택합니다. 흰 배경도 선택될 수 있습니다.'),
                      (f'_, mask = cv2.threshold(blurred, {max(threshold, 0)}, 255,\n'
                       + ('    cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)' if threshold == -1
                          else '    cv2.THRESH_BINARY_INV)') if gray_mode
                       else f'mask = ((S <= {sat}) | (V <= {dark})).astype(np.uint8) * 255'), 'blur'),
    }
    for key, operation, parent in [('close', 'MORPH_CLOSE', 'threshold'), ('open', 'MORPH_OPEN', 'close')]:
        size = options.get(f'{key}_ksize', 0)
        effect = '물체 내부의 작은 틈을 메웁니다.' if key == 'close' else '작은 흰 잡음을 제거합니다. 가는 부품도 지워질 수 있습니다.'
        descriptions[key] = (f'{key.upper()} · {size}×{size}' if size else f'{key.upper()} · 꺼짐',
                             effect if size else '이 연산을 건너뜁니다. 입력 영상과 동일합니다.',
                             f'mask = cv2.morphologyEx(mask, cv2.{operation},\n'
                             f'    np.ones(({size}, {size}), np.uint8))' if size else '# 연산 생략', parent)
    steps = [ProcessingStep(key, *descriptions[key], image) for key, image in captured.items()]
    detections = find_objects(mask, min_area_px)
    overlay = frame.copy()
    for index, det in enumerate(detections):
        cv2.polylines(overlay, [np.rint(det.box_px).astype(np.int32)], True, (0, 255, 0), 2)
        center = np.asarray(det.center_px)
        start = tuple(np.rint(center).astype(int))
        part = classify_part(det, nut_max_aspect)
        cv2.circle(overlay, start, 4, (0, 0, 255), -1)
        if part == 'screw':
            end = center + np.asarray(det.long_axis_px) * det.length_px / 2
            cv2.arrowedLine(overlay, start, tuple(np.rint(end).astype(int)), (255, 0, 0), 2)
        cv2.putText(overlay, f'{index}: {part}', start, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 160, 255), 2)
    steps.append(ProcessingStep('detections', f'검출 · {len(detections)}개',
                               f'면적 {min_area_px}px² 이상인 외곽선에 회전 사각형을 씌웁니다. '
                               '초록=박스, 빨강=중심, 파랑=나사 장축. 분류는 길이 비율 규칙입니다.',
                               'contours = cv2.findContours(mask, ...)\nrect = cv2.minAreaRect(contour)',
                               'open', overlay))
    return steps


def export_steps(steps, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    for index, step in enumerate(steps):
        path = directory / f'{index:02}_{step.key}.png'
        if not cv2.imwrite(str(path), step.image):
            raise OSError(f'이미지를 저장하지 못했습니다: {path}')
    # 번호와 코드·설명은 UTF-8 텍스트로 보존한다. 그림 폰트에 의존하지 않는다.
    text = '\n\n'.join(f'{i:02} {step.title}\n{step.description}\n{step.code}' for i, step in enumerate(steps))
    (directory / 'steps.txt').write_text(text + '\n', encoding='utf-8')
