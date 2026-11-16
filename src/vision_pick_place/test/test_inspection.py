import cv2
import numpy as np
import pytest

from vision_pick_place.camera_stream import preprocess
from vision_pick_place.inspection import export_steps, inspect_frame


@pytest.mark.parametrize('mode', ['gray', 'metal_hsv'])
def test_inspector_uses_actual_preprocessing_without_changing_input(mode, tmp_path):
    frame = np.full((100, 160, 3), (30, 160, 220), dtype=np.uint8)
    frame[30:70, 30:120] = 20
    original = frame.copy()
    options = dict(segmentation_mode=mode, close_ksize=3, open_ksize=3,
                   hsv_max_saturation=70, hsv_max_dark_value=90)
    steps = inspect_frame(frame, **options)
    stages = {step.key: step for step in steps}
    assert np.array_equal(stages['open'].image, preprocess(frame, **options))
    assert np.array_equal(frame, original)
    assert all(step.parent in stages for step in steps)
    assert ('hsv_saturation' in stages) == (mode == 'metal_hsv')
    assert stages['detections'].image.shape == frame.shape
    assert not np.array_equal(stages['detections'].image, frame)
    # 화면에 전달하는 단계 영상은 독립된 복사본이다.
    assert not np.shares_memory(stages['original'].image, frame)
    export_steps(steps, tmp_path)
    images = list(tmp_path.glob('*.png'))
    assert len(images) == len(steps)
    assert cv2.imread(str(tmp_path / '00_original.png')).shape == frame.shape
    assert 'Gaussian Blur' in (tmp_path / 'steps.txt').read_text()


def test_disabled_operations_keep_identical_masks():
    frame = np.full((100, 160, 3), 255, dtype=np.uint8)
    frame[30:70, 30:120] = 0
    stages = {step.key: step.image for step in inspect_frame(frame, blur_ksize=1)}
    assert np.array_equal(stages['threshold'], stages['close'])
    assert np.array_equal(stages['close'], stages['open'])
