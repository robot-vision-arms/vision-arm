import cv2
import numpy as np
import pytest
from vision_pick_place.team2_shaft_pose import shaft_pose


@pytest.mark.parametrize('angle',[20.,65.,86.5,115.,160.])
def test_head_does_not_move_grasp_point_off_shaft(angle):
    center=np.array([180.,180.]);a=np.radians(angle)
    R=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    mask=np.zeros((360,360),np.uint8)
    for shape in [np.array([[-60,-12],[48,-12],[48,12],[-60,12]]),
                  np.array([[48,-20],[60,-20],[60,20],[48,20]])]:
        cv2.fillConvexPoly(mask,np.rint(shape@R.T+center).astype(np.int32),255)
    point,axis=shaft_pose(mask,center)
    normal=np.array([-np.sin(a),np.cos(a)])
    assert abs((point-center)@normal)<.7
    assert abs(np.dot(axis,R[:,0]))>.999


def test_empty_mask_is_rejected():
    with pytest.raises(ValueError):shaft_pose(np.zeros((100,100),np.uint8),np.array([50.,50.]))


def test_shadow_is_not_a_part_but_bright_ring_and_screw_are_kept():
    from vision_pick_place.team2_shaft_pose import filter_metal_support
    image=np.full((160,240,3),(20,130,20),np.uint8)
    mask=np.zeros(image.shape[:2],np.uint8)
    cv2.rectangle(mask,(10,20),(40,70),255,-1);image[mask>0]=(20,20,20)
    cv2.rectangle(mask,(80,20),(95,90),255,-1);image[20:91,80:96]=(210,210,210)
    cv2.circle(mask,(170,55),22,255,-1);cv2.circle(mask,(170,55),10,0,-1)
    cv2.circle(image,(170,55),22,(210,210,210),-1);cv2.circle(image,(170,55),10,(20,130,20),-1)
    objects=[{'id':i,'u_px':x,'v_px':y} for i,(x,y) in enumerate([(25,45),(87,55),(170,55)])]
    assert [o['id'] for o in filter_metal_support(image,mask,objects)]==[1,2]
