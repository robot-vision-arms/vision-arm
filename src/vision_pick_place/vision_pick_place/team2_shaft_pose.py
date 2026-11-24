"""Image-only shaft-center refinement for screws with enlarged heads."""
import cv2
import numpy as np


def shaft_pose(mask, approximate_center):
    count,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
    candidates=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>100]
    if not candidates:raise ValueError('나사 마스크 성분이 없습니다.')
    label=min(candidates,key=lambda i:float(np.linalg.norm(centers[i]-approximate_center)))
    ys,xs=np.nonzero(labels==label);points=np.column_stack([xs,ys]).astype(np.float64)
    mean=points.mean(axis=0);_,vectors=np.linalg.eigh(np.cov((points-mean).T));axis=vectors[:,-1]
    projection=(points-mean)@axis
    lo,hi=np.quantile(projection,[.25,.75]);middle=points[(projection>=lo)&(projection<=hi)]
    # 넓은 머리를 제외한 몸통 중간 절반으로 축을 구한다. TF·모델 정답은 입력받지 않는다.
    center=middle.mean(axis=0);_,vectors=np.linalg.eigh(np.cov((middle-center).T));axis=vectors[:,-1]
    if axis[0]<0:axis=-axis
    normal=np.array([-axis[1],axis[0]])
    ends=points@axis
    center=axis*((ends.min()+ends.max())/2)+normal*(center@normal)
    return center,axis


def filter_metal_support(frame, mask, objects, *, max_saturation=70, min_value=90, min_fraction=.2):
    """Generated metallic parts must contain bright, low-chroma pixels; shadows do not.

    This is an image-only guard for Team 2 generated scenes, not a TF/known-position filter.
    """
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    count,labels,stats,centroids=cv2.connectedComponentsWithStats(mask)
    if count<=1:return []
    bright=(hsv[:,:,1]<=max_saturation)&(hsv[:,:,2]>=min_value)
    supported=np.bincount(labels[bright],minlength=count)
    result=[]
    for obj in objects:
        center=np.array([obj['u_px'],obj['v_px']])
        x,y=np.rint(center).astype(int)
        label=labels[y,x] if 0<=y<labels.shape[0] and 0<=x<labels.shape[1] else 0
        # 너트 중심은 구멍일 수 있으므로 배경 픽셀일 때 가까운 부품 성분을 찾는다.
        if label==0:label=1+np.argmin(np.sum((centroids[1:]-center)**2,axis=1))
        if supported[label]/max(1,stats[label,cv2.CC_STAT_AREA])>=min_fraction:
            result.append(obj)
    return result
