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
        # TODO: image_dir 이면 파일 목록 로드, 아니면 cv2.VideoCapture(camera_url or camera_index) 열기
        #       네트워크 영상은 끊길 수 있으니 read() 실패 시 재연결

    def read(self) -> np.ndarray | None:
        """BGR 프레임 1장을 반환. 더 읽을 수 없으면 None."""
        raise NotImplementedError

    def release(self) -> None:
        raise NotImplementedError


def preprocess(frame_bgr: np.ndarray, blur_ksize: int = 5, threshold: int = -1) -> np.ndarray:
    """그레이스케일 → Gaussian Blur → 이진화. 물체가 흰색(255)인 마스크를 반환.

    threshold 가 -1 이면 Otsu 로 자동 결정한다. 배경이 물체보다 밝으면 THRESH_BINARY_INV 를 쓴다.
    """
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    # TODO: cv2.GaussianBlur, cv2.threshold(... THRESH_OTSU ...), 필요하면 morphology open/close
    raise NotImplementedError
