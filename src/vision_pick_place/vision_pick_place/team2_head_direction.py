"""장축(180도 주기)과 별도로 나사 머리가 향하는 쪽(360도)을 추정한다."""
import math
import cv2
import numpy as np


def head_direction(det, mask, calibrator, height_mm=0.):
    axis=np.asarray(det.long_axis_px,dtype=float)
    norm=np.linalg.norm(axis)
    if not calibrator.status.ready or not np.isfinite(norm) or norm<=0:return None
    axis/=norm;normal=np.array([-axis[1],axis[0]])
    center=np.asarray(det.center_px,dtype=float)
    # OBB 안에서 양 끝의 폭을 비교한다. 장축 각도만으로는 머리/끝을 구별할 수 없다.
    x=np.linspace(-.48,.48,128)*det.length_px
    y=np.linspace(-.55,.55,64)*det.width_px
    coords=center+axis*x[None,:,None]+normal*y[:,None,None]
    patch=cv2.remap(mask,coords[:,:,0].astype(np.float32),coords[:,:,1].astype(np.float32),
                    cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT)
    widths=np.count_nonzero(patch,axis=0).astype(float)*det.width_px*1.1/64
    minus=float(np.percentile(widths[4:30],85))
    plus=float(np.percentile(widths[98:124],85))
    shaft=float(np.median(widths[45:83]))
    large,small=max(minus,plus),min(minus,plus)
    if shaft<1 or large<1.18*shaft or large<1.18*small or large-small<1.5:
        return None  # 가림·대칭 외형에서는 방향을 지어내지 않는다.
    direction=axis if plus>minus else -axis
    points=calibrator.px_to_mm(np.array([center,center+direction*det.length_px/2]),height_mm)
    delta=points[1]-points[0]
    if not np.all(np.isfinite(delta)) or np.linalg.norm(delta)<1e-6:return None
    return float(math.degrees(math.atan2(delta[1],delta[0]))%360.)
