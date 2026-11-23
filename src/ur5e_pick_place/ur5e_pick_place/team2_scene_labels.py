"""Project known Gazebo geometry into camera pixels, independently of detections."""
import math
from urllib.parse import urlparse, unquote
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def pose_matrix(pose):
    x,y,z,r,p,w=pose
    cr,sr,cp,sp,cw,sw=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(w),math.sin(w)
    R=np.array([[cw,-sw,0.],[sw,cw,0.],[0.,0.,1.]]) @ np.array([[cp,0.,sp],[0.,1.,0.],[-sp,0.,cp]]) @ np.array([[1.,0.,0.],[0.,cr,-sr],[0.,sr,cr]])
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=[x,y,z];return T


def quaternion_matrix(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def visible_vertices(model_path):
    root=ET.parse(model_path).getroot();vertices=[]
    for link in root.findall('model/link'):
        T=pose_matrix(list(map(float,link.findtext('pose','0 0 0 0 0 0').split())))
        for visual in link.findall('visual'):
            if float(visual.findtext('transparency','0'))>=1:continue
            V=pose_matrix(list(map(float,visual.findtext('pose','0 0 0 0 0 0').split())))
            mesh=visual.find('geometry/mesh/uri');cylinder=visual.find('geometry/cylinder')
            if mesh is not None:
                path=Path(unquote(urlparse(mesh.text).path))
                points=[list(map(float,line.split()[1:4])) for line in path.read_text().splitlines() if line.startswith('v ')]
            elif cylinder is not None:
                radius=float(cylinder.findtext('radius'));half=float(cylinder.findtext('length'))/2
                points=[[radius*math.cos(a),radius*math.sin(a),z] for a in np.linspace(0,2*math.pi,64,endpoint=False) for z in [-half,half]]
            else:raise ValueError('Unsupported visual geometry for labels')
            points=np.asarray(points);points=np.column_stack([points,np.ones(len(points))])
            vertices.extend(((T@V@points.T).T)[:,:3])
    return np.asarray(vertices)


def project_world(points, camera_pose, K):
    camera=pose_matrix(camera_pose)
    local=(camera[:3,:3].T@(np.asarray(points)-camera[:3,3]).T).T
    if np.any(local[:,0]<=.05):raise ValueError('Geometry behind the camera near plane')
    # Gazebo 카메라: +X 정면, -Y 영상 오른쪽, -Z 영상 아래쪽.
    return np.column_stack([K[0,2]-K[0,0]*local[:,1]/local[:,0],K[1,2]-K[1,1]*local[:,2]/local[:,0]])
