"""Bounded NDJSON transport shared by the bridge and TCP tests."""
import math
import socket
import time

from vision_pick_place.packet import decode, encode

MAX_LINE = 1024 * 1024
# 서버의 5초 유휴 제한 전에 연결을 교체한다. 영상 중단 후 첫 좌표도 새 연결로 보낸다.
IDLE_RECONNECT_S = 4.0


def validate_pose(packet):
    if not isinstance(packet, dict):
        raise ValueError('packet must be an object')
    if type(packet.get('seq')) is not int or packet['seq'] < 0:
        raise ValueError('seq must be a nonnegative integer')
    def finite(value):
        return type(value) in (int, float) and math.isfinite(value)
    if not finite(packet.get('t_capture')) or packet['t_capture'] < 0:
        raise ValueError('t_capture must be finite and nonnegative')
    if type(packet.get('calib_ok')) is not bool or not isinstance(packet.get('objects'), list):
        raise ValueError('calib_ok boolean and objects array required')
    ids = set()
    for obj in packet['objects']:
        if not isinstance(obj, dict) or type(obj.get('id')) is not int or obj['id'] < 0:
            raise ValueError('object id required')
        if obj['id'] in ids:
            raise ValueError('duplicate object id')
        ids.add(obj['id'])
        if not isinstance(obj.get('part_type'), str) or not obj['part_type']:
            raise ValueError('part_type required')
        for key in ('x_mm', 'y_mm', 'angle_deg'):
            if key not in obj or (obj[key] is not None and not finite(obj[key])):
                raise ValueError(f'{key} must be finite or null')
        if packet['calib_ok'] and not all(finite(obj[k]) for k in ('x_mm', 'y_mm')):
            raise ValueError('calibrated coordinates required')
        if not packet['calib_ok'] and any(obj[k] is not None for k in ('x_mm', 'y_mm', 'angle_deg')):
            raise ValueError('uncalibrated coordinates must be null')
        if obj['angle_deg'] is not None and not 0 <= obj['angle_deg'] < 180:
            raise ValueError('angle_deg must be in [0,180)')
    return packet


class PoseClient:
    def __init__(self, host, port, timeout=1.):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be positive')
        self.host, self.port, self.timeout = host, port, timeout
        self.sock = None
        self.buffer = b''
        self.last_transaction = {}
        self.last_ack_at = None

    def close(self):
        if self.sock is not None:
            self.sock.close()
        self.sock = None
        self.buffer = b''
        self.last_ack_at = None

    def exchange(self, packet):
        self.last_transaction = dict(stage='validating', validated=False, connected=False,
                                     send_completed=False, acked=False, tcp_roundtrip_ms=None)
        validate_pose(packet)
        raw = encode(packet)
        if len(raw) > MAX_LINE:
            raise ValueError('packet too large')
        self.last_transaction['validated'] = True
        # 아직 보내지 않은 새 프레임만 새 연결로 보낸다. ACK가 불명확한 프레임은 재전송하지 않는다.
        if self.last_ack_at is not None and time.monotonic() - self.last_ack_at >= IDLE_RECONNECT_S:
            self.close()
        deadline = time.monotonic() + self.timeout
        try:
            self.last_transaction['stage'] = 'connecting'
            if self.sock is None:
                self.sock = socket.create_connection((self.host, self.port), self.timeout)
            self.last_transaction.update(connected=True, stage='sending')
            self.sock.settimeout(max(.001, deadline-time.monotonic()))
            send_started = time.monotonic()
            self.sock.sendall(raw)
            self.last_transaction.update(send_completed=True, stage='waiting_ack')
            while b'\n' not in self.buffer:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('ACK timeout')
                self.sock.settimeout(remaining)
                chunk = self.sock.recv(4096)
                if not chunk:
                    raise ConnectionError('server disconnected before ACK')
                self.buffer += chunk
                if len(self.buffer) > MAX_LINE:
                    raise ValueError('ACK too large')
            line, self.buffer = self.buffer.split(b'\n', 1)
            ack = decode(line)
            if (not isinstance(ack, dict) or type(ack.get('ack')) is not int
                    or ack['ack'] != packet['seq'] or ack.get('status') != 'ok'):
                raise ValueError('ACK sequence or status mismatch')
            self.last_transaction.update(acked=True, stage='acked',
                tcp_roundtrip_ms=(time.monotonic()-send_started)*1000)
            self.last_ack_at = time.monotonic()
            return ack
        except Exception:
            # No automatic replay: the server may already have accepted this packet.
            self.close()
            raise
