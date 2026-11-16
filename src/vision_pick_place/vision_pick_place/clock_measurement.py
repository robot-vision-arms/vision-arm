"""Detect wall-clock discontinuities observed by the bridge, without changing source nodes.

This cannot prove clock stability before the bridge's observation interval, or
clock synchronization between different PCs. RTT uses a monotonic clock instead.
"""
import math
import time


class ClockMeasurement:
    def __init__(self, threshold_s=.05, max_sample_span_s=.005, wall=None, mono=None):
        self.wall = wall or time.time
        self.mono = mono or time.monotonic
        self.threshold = threshold_s
        self.max_span = max_sample_span_s
        self.previous = self.sample()

    def sample(self):
        before = self.mono()
        wall = self.wall()
        after = self.mono()
        return dict(wall=wall, mono=(before+after)/2, span=after-before,
                    offset=wall-(before+after)/2)

    def begin(self):
        sample = self.sample()
        previous = self.previous
        self.previous = sample
        return previous, sample

    def finish(self, token, t_capture):
        previous, start = token
        end = self.sample()
        self.previous = end
        jumps = [start['offset']-previous['offset'], end['offset']-start['offset']]
        jump = max(jumps, key=abs)
        warning = ''
        if max(previous['span'], start['span'], end['span']) > self.max_span:
            warning = 'Clock sampling was descheduled; wall-clock latency uncertain'
        elif abs(jump) > self.threshold:
            warning = 'Wall-clock discontinuity observed by bridge'
        raw = None
        if type(t_capture) in (int, float) and math.isfinite(t_capture):
            raw = (end['wall']-t_capture)*1000
            if raw < 0:
                warning = warning or 'Capture timestamp is in the future'
        else:
            warning = warning or 'Capture timestamp missing or invalid'
        return dict(latency_ms=raw if not warning else None, raw_latency_ms=raw,
                    latency_valid=not bool(warning), clock_warning=warning,
                    clock_jump_ms=jump*1000, clock_sample_span_ms=end['span']*1000,
                    clock_scope='Bridge observations only; source and cross-PC clocks not verified')
