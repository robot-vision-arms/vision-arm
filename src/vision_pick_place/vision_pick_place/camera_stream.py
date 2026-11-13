"""영상 수집과 전처리 (1주차).

스마트폰 IP 카메라(또는 USB 카메라, 이미지 폴더)에서 프레임을 읽고, 검출 전에 필요한 Blur → 이진화까지 담당한다.
ROS 에 의존하지 않으므로 노드 없이 단독으로 테스트할 수 있다.
"""
from pathlib import Path

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


def preprocess(frame_bgr: np.ndarray, blur_ksize: int = 5, threshold: int = -1) -> np.ndarray:
    """그레이스케일 → Gaussian Blur → 이진화. 물체가 흰색(255)인 마스크를 반환.

    threshold 가 -1 이면 Otsu 로 자동 결정한다. 배경이 물체보다 밝으면 THRESH_BINARY_INV 를 쓴다.
    """
    if blur_ksize < 1 or blur_ksize % 2 == 0:
        raise ValueError('blur_ksize는 1 이상의 홀수여야 합니다.')
    if threshold != -1 and not 0 <= threshold <= 255:
        raise ValueError('threshold는 -1(Otsu) 또는 0~255여야 합니다.')
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 0)
    # 밝은 작업면 위의 어두운 부품을 흰색 전경으로 만든다.
    mode = cv2.THRESH_BINARY_INV
    if threshold == -1:
        mode |= cv2.THRESH_OTSU
    _, mask = cv2.threshold(blurred, max(threshold, 0), 255, mode)
    return mask
