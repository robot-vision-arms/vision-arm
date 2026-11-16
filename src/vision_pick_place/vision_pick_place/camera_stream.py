"""영상 수집과 전처리 (1주차).

스마트폰 IP 카메라(또는 USB 카메라, 이미지 폴더)에서 프레임을 읽고, 검출 전에 필요한 Blur → 이진화까지 담당한다.
ROS 에 의존하지 않으므로 노드 없이 단독으로 테스트할 수 있다.
"""
from pathlib import Path
from typing import Callable

import cv2
import numpy as np


class FrameSource:
    """스마트폰 영상 주소, 카메라 번호, 이미지 폴더를 같은 인터페이스로 감싼다.

    우선순위: image_dir → camera_url → camera_index.
    image_dir 가 주어지면 폴더 안 이미지를 이름순으로 반복 재생한다 (카메라 없이 MVTec/촬영본 테스트용).
    camera_url 은 폰 IP 카메라 앱의 영상 주소로, cv2.VideoCapture 가 URL 을 그대로 연다.
    """

    def __init__(self, camera_index: int = 0, image_dir: str = '', camera_url: str = ''):
        self.camera_index = camera_index
        self.camera_url = camera_url
        self.image_dir = Path(image_dir) if image_dir else None
        self._cap = None
        self._closed = False
        self._index = 0
        self._images = []
        if self.image_dir is not None:
            if not self.image_dir.is_dir():
                raise FileNotFoundError(f'이미지 폴더가 없습니다: {self.image_dir}')
            extensions = {'.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff'}
            self._images = sorted(
                p for p in self.image_dir.iterdir()
                if p.is_file() and p.suffix.lower() in extensions
            )
            if not self._images:
                raise ValueError(f'이미지 파일이 없습니다: {self.image_dir}')
        else:
            self._open_camera()

    def _open_camera(self) -> None:
        if self._cap is not None:
            self._cap.release()
        if self.camera_url:
            self._cap = cv2.VideoCapture(
                self.camera_url, cv2.CAP_FFMPEG,
                [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
                 cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000],
            )
        else:
            self._cap = cv2.VideoCapture(self.camera_index)

    def read(self) -> np.ndarray | None:
        """BGR 프레임 1장을 반환. 더 읽을 수 없으면 None."""
        if self._closed:
            return None
        if self.image_dir is not None:
            # 손상된 파일은 건너뛰되, 모두 손상된 경우 무한 반복하지 않는다.
            for _ in self._images:
                path = self._images[self._index]
                self._index = (self._index + 1) % len(self._images)
                frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
                if frame is not None:
                    return frame
            return None
        ok, frame = self._cap.read()
        if not ok and self.camera_url:
            self._open_camera()
            ok, frame = self._cap.read()
        return frame if ok else None

    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()
        self._closed = True


def preprocess(frame_bgr: np.ndarray, blur_ksize: int = 5, threshold: int = -1, *,
               segmentation_mode: str = 'gray', hsv_max_saturation: int = 70,
               hsv_max_dark_value: int = 70, close_ksize: int = 0, open_ksize: int = 0,
               on_step: Callable[[str, np.ndarray], None] | None = None) -> np.ndarray:
    """그레이스케일 → Gaussian Blur → 이진화. 물체가 흰색(255)인 마스크를 반환.

    threshold 가 -1 이면 Otsu 로 자동 결정한다. 배경이 물체보다 밝으면 THRESH_BINARY_INV 를 쓴다.
    metal_hsv는 색이 있는 배경에서 저채도 금속·어두운 부품을 선택하는 대체 방식이다.
    이 모드에서는 threshold를 사용하지 않는다. 단색·흰 배경에는 gray를 사용한다.
    close/open 커널은 0이면 끈다. 형태 연산은 close 다음 open 순서로 적용한다.
    on_step을 주면 실제 연산의 중간 영상을 전달한다. 콜백은 영상을 수정하지 않아야 한다.
    """
    if blur_ksize < 1 or blur_ksize % 2 == 0:
        raise ValueError('blur_ksize는 1 이상의 홀수여야 합니다.')
    if threshold != -1 and not 0 <= threshold <= 255:
        raise ValueError('threshold는 -1(Otsu) 또는 0~255여야 합니다.')
    for name, size in (('close_ksize', close_ksize), ('open_ksize', open_ksize)):
        if size < 0 or (size > 0 and size % 2 == 0):
            raise ValueError(f'{name}는 0 또는 양의 홀수여야 합니다.')
    if not 0 <= hsv_max_saturation <= 255 or not 0 <= hsv_max_dark_value <= 255:
        raise ValueError('HSV 채도·명도 기준은 0~255여야 합니다.')
    def record(name, image):
        if on_step is not None:
            on_step(name, image)

    record('original', frame_bgr)
    if segmentation_mode == 'gray':
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        record('grayscale', gray)
        blurred = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 0)
        record('blur', blurred)
        mode = cv2.THRESH_BINARY_INV
        if threshold == -1:
            mode |= cv2.THRESH_OTSU
        _, mask = cv2.threshold(blurred, max(threshold, 0), 255, mode)
    elif segmentation_mode == 'metal_hsv':
        blurred = cv2.GaussianBlur(frame_bgr, (blur_ksize, blur_ksize), 0)
        record('blur', blurred)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        if on_step is not None:
            record('hsv_saturation', hsv[:, :, 1])
            record('hsv_value', hsv[:, :, 2])
        foreground = (hsv[:, :, 1] <= hsv_max_saturation) | (hsv[:, :, 2] <= hsv_max_dark_value)
        mask = foreground.astype(np.uint8) * 255
    else:
        raise ValueError('segmentation_mode는 gray 또는 metal_hsv여야 합니다.')
    record('threshold', mask)
    for name, operation, size in (('close', cv2.MORPH_CLOSE, close_ksize),
                                  ('open', cv2.MORPH_OPEN, open_ksize)):
        if size:
            mask = cv2.morphologyEx(mask, operation, np.ones((size, size), np.uint8))
        record(name, mask)
    return mask
