import cv2
import numpy as np
import pytest

from vision_pick_place.camera_stream import FrameSource, preprocess


def test_folder_order_repeat_corrupt_file_and_release(tmp_path):
    cv2.imwrite(str(tmp_path / 'b.png'), np.full((8, 8, 3), 200, np.uint8))
    cv2.imwrite(str(tmp_path / 'a.png'), np.full((8, 8, 3), 50, np.uint8))
    (tmp_path / 'c.png').write_text('broken image')
    source = FrameSource(image_dir=str(tmp_path), camera_url='invalid://unused')
    assert [int(source.read()[0, 0, 0]) for _ in range(4)] == [50, 200, 50, 200]
    source.release()
    source.release()
    assert source.read() is None


def test_folder_errors_and_all_corrupt(tmp_path):
    with pytest.raises(FileNotFoundError):
        FrameSource(image_dir=str(tmp_path / 'missing'))
    with pytest.raises(ValueError):
        FrameSource(image_dir=str(tmp_path))
    (tmp_path / 'bad.png').write_text('broken image')
    source = FrameSource(image_dir=str(tmp_path))
    assert source.read() is None
    source.release()


@pytest.mark.parametrize('threshold', [-1, 127])
def test_dark_part_becomes_white_on_black_background(threshold):
    frame = np.full((100, 100, 3), 240, np.uint8)
    frame[30:70, 40:60] = 20
    mask = preprocess(frame, threshold=threshold)
    assert mask.shape == (100, 100)
    assert mask.dtype == np.uint8
    assert set(np.unique(mask)) == {0, 255}
    assert mask[50, 50] == 255
    assert mask[0, 0] == 0


@pytest.mark.parametrize('kwargs', [{'blur_ksize': 0}, {'blur_ksize': 4}, {'threshold': 256}])
def test_invalid_preprocess_settings(kwargs):
    with pytest.raises(ValueError):
        preprocess(np.zeros((10, 10, 3), np.uint8), **kwargs)
