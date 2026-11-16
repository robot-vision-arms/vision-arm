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


def test_hsv_keeps_metal_and_dark_parts_on_colored_background():
    frame = np.full((100, 160, 3), (30, 160, 220), np.uint8)
    frame[20:80, 20:40] = (200, 200, 200)  # 밝은 금속
    frame[20:80, 70:90] = (10, 20, 50)     # 색이 있지만 어두운 부품
    mask = preprocess(frame, blur_ksize=1, segmentation_mode='metal_hsv',
                      hsv_max_saturation=70, hsv_max_dark_value=90)
    assert mask[50, 30] == mask[50, 80] == 255
    assert mask[5, 5] == 0
    assert mask.dtype == np.uint8 and set(np.unique(mask)) == {0, 255}


def test_morphology_fills_small_gap_and_removes_isolated_noise():
    frame = np.full((60, 80, 3), 255, np.uint8)
    frame[15:45, 20:60] = 0
    frame[15:45, 39:40] = 255
    frame[5, 5] = 0
    mask = preprocess(frame, blur_ksize=1, threshold=127, close_ksize=3, open_ksize=3)
    assert mask[30, 39] == 255
    assert mask[5, 5] == 0
    assert mask[30, 25] == 255


@pytest.mark.parametrize('kwargs', [
    {'segmentation_mode': 'missing'}, {'close_ksize': -1}, {'close_ksize': 2},
    {'open_ksize': 4}, {'hsv_max_saturation': 256}, {'hsv_max_dark_value': -1},
])
def test_invalid_segmentation_settings_are_rejected(kwargs):
    with pytest.raises(ValueError):
        preprocess(np.zeros((10, 10, 3), np.uint8), **kwargs)


class _FakeCapture:
    """초당 수백 장을 내보내는 가짜 카메라. 프레임 값 [0,0,0] 에 몇 번째 프레임인지 적는다."""
    opened = 0

    def __init__(self, *args, fail_after=None, **kwargs):
        _FakeCapture.opened += 1
        self.count = 0
        self.fail_after = fail_after
        self.released = False

    def read(self):
        import time
        time.sleep(0.002)
        if self.released or (self.fail_after is not None and self.count >= self.fail_after):
            return False, None
        self.count += 1
        frame = np.zeros((4, 4, 3), np.uint8)
        frame[0, 0, 0] = self.count % 256
        frame[0, 0, 1] = self.count // 256
        return True, frame

    def release(self):
        self.released = True


def _frame_no(frame):
    return int(frame[0, 0, 0]) + 256 * int(frame[0, 0, 1])


def test_live_source_returns_latest_frame_not_backlog(monkeypatch):
    import time
    monkeypatch.setattr(cv2, 'VideoCapture', _FakeCapture)
    source = FrameSource(camera_url='rtsp://fake/live', read_timeout_s=1.0)
    first = _frame_no(source.read())
    time.sleep(0.3)  # 노드가 다른 일을 하는 동안 카메라는 계속 프레임을 보낸다
    second = _frame_no(source.read())
    # 버퍼를 한 장씩 꺼내면 first+1 이 나오지만, 최신 프레임만 쓰므로 훨씬 뒤 번호가 나와야 한다
    assert second > first + 20
    source.release()
    assert source.read() is None
    assert not source._thread.is_alive()


def test_live_source_reconnects_after_stream_drop(monkeypatch):
    _FakeCapture.opened = 0
    caps = []

    def factory(*args, **kwargs):
        cap = _FakeCapture(fail_after=5 if not caps else None)
        caps.append(cap)
        return cap

    monkeypatch.setattr(cv2, 'VideoCapture', factory)
    import time
    source = FrameSource(camera_url='rtsp://fake/live', read_timeout_s=2.0)
    time.sleep(1.0)  # 첫 연결은 5장 뒤 끊기고, 0.5초 쉰 뒤 다시 연결된다
    assert len(caps) >= 2
    assert source.read() is not None  # 다시 연결된 스트림의 프레임
    assert caps[-1].count > 0
    source.release()


def test_live_source_read_times_out_when_no_frames(monkeypatch):
    monkeypatch.setattr(cv2, 'VideoCapture', lambda *a, **k: _FakeCapture(fail_after=0))
    source = FrameSource(camera_url='rtsp://fake/live', read_timeout_s=0.3)
    assert source.read() is None
    source.release()
