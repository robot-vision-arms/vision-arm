"""Team 2 communication measurements; independent of ROS and GUI."""
import math


def percentile95(values):
    values = sorted(values)
    if not values:
        return None
    index = .95 * (len(values)-1)
    lo = int(index)
    return values[lo] + (values[min(lo+1, len(values)-1)]-values[lo]) * (index-lo)


class CommunicationCheck:
    def __init__(self, count=100, require_image=True):
        self.count, self.require_image = count, require_image
        self.packets, self.targets, self.logs = {}, {}, {}
        self.errors = []
        self.image_received = False
        self.duplicate_packets = 0

    def receive(self, kind, packet):
        if not isinstance(packet, dict) or type(packet.get('seq')) is not int:
            self.errors.append(f'{kind}: invalid seq or packet')
            return
        seq = packet['seq']
        if kind == 'packet':
            if seq in self.packets:
                self.duplicate_packets += 1
                self.errors.append(f'duplicate source seq: {seq}')
            elif len(self.packets) < self.count:
                self.packets[seq] = packet
        elif kind == 'target':
            self.targets[seq] = packet
        elif kind == 'log':
            self.logs[seq] = packet
            if packet.get('error') or packet.get('acked') is not True:
                self.errors.append(f'TCP seq {seq}: {packet.get("error") or "ACK unsuccessful"}')

    def report(self):
        expected = set(self.packets)
        acked = {s for s in expected if self.logs.get(s, {}).get('acked') is True
                 and not self.logs[s].get('error')}
        identical = {s for s in expected if self.targets.get(s) == self.packets[s]}
        mismatches = sorted(s for s in expected & self.targets.keys()
                            if self.targets[s] != self.packets[s])
        latencies = [self.logs[s].get('latency_ms') for s in acked
                     if self.logs[s].get('latency_valid', True)]
        clock_warnings = [dict(seq=s, warning=self.logs[s].get('clock_warning', 'Latency marked invalid'))
                          for s in sorted(acked) if not self.logs[s].get('latency_valid', True)]
        valid_latencies = [v for v in latencies if type(v) in (float, int) and math.isfinite(v) and v >= 0]
        errors = list(self.errors)
        if len(latencies) != len(valid_latencies):
            errors.append('Missing, negative or non-finite successful ACK latency')
        if clock_warnings:
            errors.append('Some frame-to-ACK latencies are invalid; see clock_warnings')
        if mismatches:
            errors.append('Source and robot target packets differ')
        full = len(expected) == self.count
        communication_ok = (full and len(acked) == self.count and len(identical) == self.count
                            and not self.errors and not mismatches)
        ok = (full and len(acked) == self.count and len(identical) == self.count
              and not errors and (self.image_received or not self.require_image))
        return dict(ok=ok, required_packets=self.count, observed_packets=len(expected),
                    communication_ok=communication_ok,
                    latency_ok=len(valid_latencies) == self.count and not clock_warnings,
                    acked_packets=len(acked), identical_packets=len(identical),
                    ack_success_rate=len(acked)/len(expected) if expected else 0.,
                    target_delivery_rate=len(identical)/len(expected) if expected else 0.,
                    latency_p95_ms=percentile95(valid_latencies),
                    latency_samples=len(valid_latencies), errors=errors,
                    clock_warnings=clock_warnings,
                    tcp_roundtrip_p95_ms=percentile95([
                        self.logs[s]['tcp_roundtrip_ms'] for s in acked
                        if type(self.logs[s].get('tcp_roundtrip_ms')) in (int, float)
                        and math.isfinite(self.logs[s]['tcp_roundtrip_ms'])
                        and self.logs[s]['tcp_roundtrip_ms'] >= 0]),
                    missing_acks=sorted(expected-acked), missing_or_changed_targets=sorted(expected-identical),
                    image_received=self.image_received, image_required=self.require_image,
                    calib_ok_values=sorted({p.get('calib_ok') for p in self.packets.values()
                                           if type(p.get('calib_ok')) is bool}),
                    robot_motion_commanded=False,
                    measurement='First N observed source packets; frame-to-ACK wall-clock latency')
