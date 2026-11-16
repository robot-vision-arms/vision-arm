import pytest

from vision_pick_place.clock_measurement import ClockMeasurement


class Clock:
    wall = 1000.
    mono = 0.

    def advance(self, elapsed, jump=0):
        self.wall += elapsed+jump
        self.mono += elapsed


def make_clock():
    fake = Clock()
    return fake, ClockMeasurement(wall=lambda: fake.wall, mono=lambda: fake.mono)


def test_stable_frame_latency_is_preserved():
    fake, clock = make_clock()
    fake.advance(.1)
    token = clock.begin()
    fake.advance(.05)
    result = clock.finish(token, 1000.)
    assert result['latency_valid']
    assert result['latency_ms'] == pytest.approx(150)
    assert result['clock_jump_ms'] == pytest.approx(0)


@pytest.mark.parametrize('jump', [.7, -.7])
def test_wall_jump_during_transaction_excludes_latency(jump):
    fake, clock = make_clock()
    token = clock.begin()
    fake.advance(.1, jump)
    result = clock.finish(token, 999.)
    assert result['latency_ms'] is None
    assert not result['latency_valid']
    assert result['clock_jump_ms'] == pytest.approx(jump*1000)
    assert result['clock_warning']


def test_jump_between_frames_is_detected_then_next_stable_frame_recovers():
    fake, clock = make_clock()
    fake.advance(.1, .8)
    result = clock.finish(clock.begin(), fake.wall)
    assert not result['latency_valid']
    fake.advance(.1)
    assert clock.finish(clock.begin(), fake.wall)['latency_valid']


def test_future_capture_is_not_reported_as_valid_negative_latency():
    fake, clock = make_clock()
    assert not clock.finish(clock.begin(), fake.wall+10)['latency_valid']


def test_slow_clock_sampling_is_marked_uncertain_not_a_definite_jump():
    fake = Clock()
    def mono():
        fake.mono += .01
        return fake.mono
    clock = ClockMeasurement(wall=lambda: fake.wall, mono=mono)
    result = clock.finish(clock.begin(), fake.wall)
    assert not result['latency_valid']
    assert 'descheduled' in result['clock_warning']
