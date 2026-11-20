"""ACK-before-source ordering across startup must not create false packet loss."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize('settle,excluded', [(0,0),(2,1)])
def test_delayed_source_crossing_startup_boundary(monkeypatch,tmp_path,settle,excluded):
    scripts=Path(__file__).resolve().parents[3]/'scripts'
    monkeypatch.syspath_prepend(str(scripts))
    spec=importlib.util.spec_from_file_location('boundary_checker',scripts/'check_team_integration.py')
    checker=importlib.util.module_from_spec(spec);spec.loader.exec_module(checker)
    callbacks={};tick=[0]
    class FakeNode:
        def create_subscription(self,kind,topic,callback,depth):
            callbacks[topic]=callback
            return SimpleNamespace(get_publisher_count=lambda:1)
        def destroy_node(self):pass
    def emit(topic,value):callbacks[topic](SimpleNamespace(data=json.dumps(value)))
    def ack(seq):return dict(seq=seq,acked=True,error='',latency_valid=True,latency_ms=1.)
    def packet(seq):return dict(seq=seq,t_capture=0.,calib_ok=False,objects=[])
    def spin(node,timeout_sec):
        tick[0]+=1
        if tick[0]==1:
            emit('/robot/log',ack(1));emit('/robot/targets',packet(1))
        elif tick[0]==4:
            emit('/vision/detections',packet(1))
        elif tick[0] in [5,6]:
            seq=tick[0]-3
            emit('/vision/detections',packet(seq));emit('/robot/targets',packet(seq));emit('/robot/log',ack(seq))
            callbacks['/vision/debug_image'](SimpleNamespace(width=1,height=1,data=[1]))
    monkeypatch.setattr(checker,'rclpy',SimpleNamespace(init=lambda args:None,
        create_node=lambda name:FakeNode(),spin_once=spin,try_shutdown=lambda:None))
    monkeypatch.setattr(checker,'time',SimpleNamespace(monotonic=lambda:float(tick[0])))
    output=tmp_path/'report.json'
    monkeypatch.setattr('sys.argv',['check','--count','2','--settle-s',str(settle),'--timeout','20','--out',str(output)])
    assert checker.main()==0
    report=json.loads(output.read_text())
    assert report['acked_packets']==report['identical_packets']==2
    assert report['boundary_excluded_source_packets']==excluded
    assert report['errors']==[]
