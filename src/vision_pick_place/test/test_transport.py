import json
import socket
from threading import Thread
import time

import pytest

from vision_pick_place.packet import encode
from vision_pick_place.robot_server import PoseServer
from vision_pick_place.transport import PoseClient, validate_pose


def pose(seq=1):
    return dict(seq=seq, t_capture=time.time(), calib_ok=False, objects=[])


@pytest.fixture
def server(tmp_path):
    forwarded = []
    srv = PoseServer(('127.0.0.1', 0), forwarded.append, tmp_path)
    thread = Thread(target=srv.serve_forever, kwargs={'poll_interval': .01}, daemon=True)
    thread.start()
    yield srv, forwarded
    srv.shutdown()
    srv.server_close()
    thread.join(2)


def test_split_and_coalesced_lines_and_invalid_recovery(server):
    srv, forwarded = server
    with socket.create_connection(srv.server_address, timeout=2) as sock:
        stream = sock.makefile('rb')
        raw = encode(pose())
        sock.sendall(raw[:10])
        sock.sendall(raw[10:]+b'invalid json\n'+encode(pose(2)))
        assert json.loads(stream.readline()) == dict(ack=1, status='ok')
        assert json.loads(stream.readline())['status'] == 'error'
        assert json.loads(stream.readline()) == dict(ack=2, status='ok')
        stream.close()
    deadline = time.monotonic()+2
    while len(forwarded) < 2 and time.monotonic() < deadline:
        time.sleep(.01)
    assert [p['seq'] for p in forwarded] == [1, 2]


def test_100_acknowledgements(server):
    srv, forwarded = server
    client = PoseClient(*srv.server_address)
    try:
        for seq in range(100):
            assert client.exchange(pose(seq))['ack'] == seq
    finally:
        client.close()


def test_bad_ack_disconnects_and_next_frame_reconnects():
    listener = socket.create_server(('127.0.0.1', 0))
    listener.settimeout(2)
    def respond():
        for seq in [999, 2]:
            with listener.accept()[0] as conn:
                with conn.makefile('rb') as stream:
                    stream.readline()
                conn.sendall(encode(dict(ack=seq, status='ok')))
    thread = Thread(target=respond, daemon=True)
    thread.start()
    client = PoseClient(*listener.getsockname())
    try:
        with pytest.raises(ValueError, match='ACK'):
            client.exchange(pose())
        assert client.sock is None
        assert client.exchange(pose(2))['ack'] == 2
    finally:
        client.close()
        listener.close()
        thread.join(2)


def test_ack_timeout_closes_connection():
    listener = socket.create_server(('127.0.0.1', 0))
    listener.settimeout(2)
    def respond():
        with listener.accept()[0] as conn:
            conn.recv(4096)
            time.sleep(.15)
    thread = Thread(target=respond, daemon=True)
    thread.start()
    client = PoseClient(*listener.getsockname(), timeout=.05)
    try:
        with pytest.raises(TimeoutError):
            client.exchange(pose())
        assert client.sock is None
    finally:
        client.close()
        listener.close()
        thread.join(2)


@pytest.mark.parametrize('update', [dict(seq=True), dict(t_capture=float('nan')),
                                   dict(calib_ok='true'), dict(objects=[{}])])
def test_invalid_packet_rejected(update):
    packet = pose()
    packet.update(update)
    with pytest.raises(ValueError):
        validate_pose(packet)


def test_calibration_contract():
    packet = pose()
    obj = dict(id=0, part_type='nut', x_mm=None, y_mm=None, angle_deg=None)
    packet['objects'] = [obj]
    validate_pose(packet)
    packet['calib_ok'] = True
    with pytest.raises(ValueError):
        validate_pose(packet)
    obj.update(x_mm=1., y_mm=2.)
    validate_pose(packet)


def test_calibrated_packet_is_forwarded_without_coordinate_changes(server):
    srv, forwarded = server
    packet = dict(seq=3, t_capture=time.time(), calib_ok=True, objects=[
        dict(id=0, part_type='screw', x_mm=102.4, y_mm=-55., angle_deg=179.5,
             u_px=640.2, v_px=360.8),
        dict(id=1, part_type='nut', x_mm=10., y_mm=20., angle_deg=None,
             u_px=100., v_px=200.)])
    client = PoseClient(*srv.server_address)
    try:
        assert client.exchange(packet)['ack'] == 3
        deadline = time.monotonic()+2
        while not forwarded and time.monotonic() < deadline:
            time.sleep(.01)
        assert forwarded == [packet]
    finally:
        client.close()


def test_actual_server_restart_requires_new_frame_and_recovers(tmp_path):
    first_packets, second_packets = [], []
    first = PoseServer(('127.0.0.1', 0), first_packets.append, tmp_path)
    address = first.server_address
    thread = Thread(target=first.serve_forever, kwargs={'poll_interval': .01}, daemon=True)
    thread.start()
    client = PoseClient(*address, timeout=.3)
    second = second_thread = None
    try:
        assert client.exchange(pose(1))['ack'] == 1
        first.shutdown()
        first.server_close()
        thread.join(2)
        with pytest.raises((OSError, ValueError)):
            client.exchange(pose(2))
        assert client.sock is None
        second = PoseServer(address, second_packets.append, tmp_path)
        second_thread = Thread(target=second.serve_forever, kwargs={'poll_interval': .01}, daemon=True)
        second_thread.start()
        assert client.exchange(pose(3))['ack'] == 3
        deadline = time.monotonic()+2
        while not second_packets and time.monotonic() < deadline:
            time.sleep(.01)
        assert [packet['seq'] for packet in second_packets] == [3]  # no replay of failed packet
    finally:
        client.close()
        if second is not None:
            second.shutdown()
            second.server_close()
            second_thread.join(2)
        if thread.is_alive():
            first.shutdown()
            first.server_close()
            thread.join(2)


def test_ack_success_does_not_hide_forward_failure(server):
    import csv
    srv, _ = server
    def fail_forward(packet):
        raise RuntimeError('ROS publisher failed')
    srv.on_packet = fail_forward
    client = PoseClient(*srv.server_address)
    try:
        assert client.exchange(pose())['status'] == 'ok'
        deadline = time.monotonic()+2
        rows = []
        while time.monotonic() < deadline:
            with srv.log_path.open() as stream:
                rows = list(csv.DictReader(stream))
            if any(row['status'] == 'forward_error' for row in rows):
                break
            time.sleep(.01)
        assert [row['status'] for row in rows] == ['ok', 'forward_error']
        assert rows[-1]['error'] == 'ROS publisher failed'
    finally:
        client.close()


def test_transaction_stages_distinguish_validation_from_ack(server):
    srv, _ = server
    client = PoseClient(*srv.server_address)
    try:
        with pytest.raises(ValueError):
            client.exchange({'seq': 'bad'})
        assert client.last_transaction['stage'] == 'validating'
        assert not client.last_transaction['send_completed']
        client.exchange(pose())
        result = client.last_transaction
        assert result['stage'] == 'acked'
        assert result['validated'] and result['connected'] and result['send_completed'] and result['acked']
        assert result['tcp_roundtrip_ms'] >= 0
    finally:
        client.close()


def test_failed_write_is_not_marked_as_send_completed(monkeypatch):
    class BrokenSocket:
        def settimeout(self, value):
            pass
        def sendall(self, data):
            raise BrokenPipeError('write failed')
        def close(self):
            pass
    monkeypatch.setattr(socket, 'create_connection', lambda *a, **kw: BrokenSocket())
    client = PoseClient('127.0.0.1', 1234)
    with pytest.raises(BrokenPipeError):
        client.exchange(pose())
    assert client.last_transaction['stage'] == 'sending'
    assert client.last_transaction['connected']
    assert not client.last_transaction['send_completed']
    assert not client.last_transaction['acked']
    assert client.sock is None
