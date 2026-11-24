from types import SimpleNamespace
import copy
import numpy as np
import pytest

from vision_pick_place.vision_detector import Detection
from vision_pick_place.team2_length_measurement import detection_length_mm,LengthTracker,process_frame_with_lengths


def detection():
    return Detection((200.,200.),np.array([[100.,190.],[300.,190.],[300.,210.],[100.,210.]]),
                     (1.,0.),200.,20.,4000.)


class Calibration:
    image_size=None
    status=SimpleNamespace(ready=True)
    def px_to_mm(self,points,height=0.):return np.asarray(points)*.1


def packet():
    return {'seq':1,'calib_ok':True,'objects':[
        dict(id=0,part_type='screw',x_mm=20.,y_mm=20.,angle_deg=0.,u_px=200.,v_px=200.),
        dict(id=1,part_type='screw',x_mm=70.,y_mm=20.,angle_deg=0.,u_px=700.,v_px=200.)]}


def test_measurement_uses_both_calibrated_endpoints():
    class Projective(Calibration):
        def px_to_mm(self,points,height=0.):
            p=np.asarray(points);return p*.1/(1+p[:,0:1]*.001)
    expected=np.linalg.norm(Projective().px_to_mm([[300.,200.],[100.,200.]])[0]-Projective().px_to_mm([[300.,200.],[100.,200.]])[1])
    assert detection_length_mm(detection(),Projective())==pytest.approx(expected)
    uncal=Calibration();uncal.status=SimpleNamespace(ready=False)
    assert detection_length_mm(detection(),uncal) is None


def test_stability_tracks_positions_when_detection_ids_change():
    tracker=LengthTracker();p=packet();original=copy.deepcopy(p)
    for _ in range(2):assert 'length_mm' not in tracker.annotate(p,{0:15.,1:35.})['objects'][0]
    p['objects'].reverse()
    for i,o in enumerate(p['objects']):o['id']=i
    measured=tracker.annotate(p,{0:35.,1:15.})
    assert [o['length_mm'] for o in measured['objects']]==[35.,15.]
    assert p['objects'][0]['x_mm']==original['objects'][1]['x_mm']
    assert all('length_mm' not in o for o in p['objects'])


def test_unstable_or_invalid_values_never_reuse_old_size():
    tracker=LengthTracker();p=packet()
    for length in [20.,20.,20.]:p=tracker.annotate(p,{0:length})
    assert p['objects'][0]['length_mm']==20.
    p=tracker.annotate(p,{0:30.})
    assert p['objects'][0]['length_status']=='unstable' and 'length_mm' not in p['objects'][0]
    p=tracker.annotate(p,{0:float('nan')})
    assert 'length_mm' not in p['objects'][0]


def test_yolo_runs_once_and_original_coordinates_are_preserved():
    class Detector:
        calls=0
        def detect(self,frame):
            self.calls+=1
            return [SimpleNamespace(det=detection(),part_type='screw',conf=.9)]
    detector=Detector();tracker=LengthTracker();frame=np.zeros((400,400,3),np.uint8)
    for i in range(3):
        result,_=process_frame_with_lengths(frame,Calibration(),i,1.,tracker,detector=detector)
    assert detector.calls==3
    obj=result['objects'][0]
    assert (obj['part_type'],obj['x_mm'],obj['y_mm'],obj['angle_deg'])==('screw',20.,20.,0.)
    assert obj['length_mm']==20. and obj['length_source']=='calibrated_obb'
