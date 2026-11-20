"""Team 2 regression: ACK waits must not silently replace received frames."""
import csv
import json
import threading
import time

import pytest
import rclpy
from std_msgs.msg import String
from vision_pick_place.robot_bridge import RobotBridge


class HeldClient:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()
        self.calls = []
        self.last_transaction = {}

    def exchange(self, packet):
        self.calls.append(packet['seq'])
        self.entered.set()
        assert self.release.wait(3)
        self.last_transaction = dict(stage='acked', send_completed=True)

    def close(self):
        pass


def packet(seq):
    return String(data=json.dumps(dict(seq=seq,t_capture=time.time(),calib_ok=False,objects=[])))


@pytest.mark.parametrize('mode', ['burst', 'overflow', 'expired'])
def test_blocked_ack_does_not_block_receiver_and_drops_are_recorded(tmp_path, mode):
    rclpy.init(args=['--ros-args','-p',f'log_dir:={tmp_path}',
                    '-p',f'receive_queue_depth:={2 if mode == "overflow" else 32}',
                    '-p',f'max_queue_age_s:={0.05 if mode == "expired" else 2.0}'])
    node = RobotBridge()
    client = HeldClient()
    node.client.close();node.client = client
    try:
        node.on_detections(packet(1))
        assert client.entered.wait(1)
        count = 20 if mode == 'burst' else 4
        started = time.monotonic()
        for seq in range(2,count+1):node.on_detections(packet(seq))
        assert time.monotonic()-started < 0.3
        if mode == 'expired':time.sleep(.1)
        client.release.set()
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            with open(node.log_file.name) as stream:
                rows=list(csv.DictReader(stream))
            if len(rows)==count:break
            time.sleep(.01)
        assert len(rows)==count
        assert len({r['seq'] for r in rows})==count
        if mode == 'burst':
            assert client.calls == list(range(1,count+1))
            assert all(r['acked']=='True' for r in rows)
        elif mode == 'overflow':
            assert client.calls == [1,3,4]
            assert next(r for r in rows if r['seq']=='2')['stage']=='queue_overflow'
        else:
            assert client.calls == [1]
            assert all(r['stage']=='queue_expired' for r in rows if r['seq']!='1')
    finally:
        client.release.set();node.destroy_node();rclpy.try_shutdown()
