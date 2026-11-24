from types import SimpleNamespace
import cv2
import numpy as np
import pytest
from vision_pick_place.team2_head_direction import head_direction


class Calibration:
    status=SimpleNamespace(ready=True)
    def px_to_mm(self,points,height=0.):return np.asarray(points)*[.2,-.2]


@pytest.mark.parametrize('flip,expected',[(False,0.),(True,180.)])
def test_head_width_resolves_half_turn_without_changing_axis(flip,expected):
    mask=np.zeros((100,160),np.uint8)
    cv2.rectangle(mask,(30,46),(130,54),255,-1)
    cv2.rectangle(mask,(110,36),(130,64),255,-1)
    if flip:mask=cv2.flip(mask,1)
    det=SimpleNamespace(long_axis_px=(1.,0.),center_px=(80.,50.),length_px=100.,width_px=30.)
    assert head_direction(det,mask,Calibration())==pytest.approx(expected)


def test_symmetric_screw_does_not_invent_head_direction():
    mask=np.zeros((100,160),np.uint8);cv2.rectangle(mask,(30,46),(130,54),255,-1)
    det=SimpleNamespace(long_axis_px=(1.,0.),center_px=(80.,50.),length_px=100.,width_px=30.)
    assert head_direction(det,mask,Calibration()) is None
