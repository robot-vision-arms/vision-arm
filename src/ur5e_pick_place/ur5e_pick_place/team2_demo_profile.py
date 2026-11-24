"""Generate an isolated ten-part scene; original models/calibration remain untouched."""
import copy
import json
import math
from pathlib import Path
import random
import shutil
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml


def scene_packet(count=10, seed=0, position_jitter_mm=0.):
    if not 1 <= count <= 10:
        raise ValueError('This A4 fixture supports 1..10 objects')
    if not math.isfinite(position_jitter_mm) or not 0 <= position_jitter_mm <= 5:
        raise ValueError('Position jitter must be 0..5 mm')
    rng=random.Random(seed)
    kinds=['screw' if k%2==0 else 'nut' for k in range(count)];rng.shuffle(kinds)
    # 5열×2행. 나사 길이 40 mm와 마커 여백을 고려해 서로 붙지 않게 배치한다.
    packet=dict(seq=seed,calib_ok=True,synthetic=True,source='team2_scene_initialization',objects=[
        dict(id=k,part_type=kind,x_mm=60.+45*(k%5),y_mm=70.+60*(k//5),
             angle_deg=rng.uniform(65,115) if kind=='screw' else None)
        for k,kind in enumerate(kinds)])
    # 마커 여백·부품 간격을 지키는 작은 위치 변동이다. 임의 밀집·겹침 배치 검증은 아니다.
    if position_jitter_mm:
        for obj in packet['objects']:
            obj['x_mm']+=rng.uniform(-position_jitter_mm,position_jitter_mm)
            obj['y_mm']+=rng.uniform(-position_jitter_mm,position_jitter_mm)
    return packet


def write_mesh(path, kind):
    vertices=[];faces=[]
    def ring(radius,z,n=48,phase=0):
        indices=[]
        for k in range(n):
            a=2*math.pi*k/n+phase;indices.append(len(vertices)+1)
            vertices.append((radius*math.cos(a),radius*math.sin(a),z))
        return indices
    def connect(a,b):
        for k in range(len(a)):
            j=(k+1)%len(a);faces.extend([(a[k],b[j],b[k]),(a[k],a[j],b[j])])
    if kind=='nut':
        # 육각 외형과 실제로 뚫린 시각적 구멍. 충돌·질량은 원본의 단순화 모델을 유지한다.
        def hex_ring(z):
            ids=[]
            for k in range(48):
                a=2*math.pi*k/48
                radial=.0075*math.cos(math.pi/6)/math.cos((a+math.pi/6)%(math.pi/3)-math.pi/6)
                ids.append(len(vertices)+1);vertices.append((radial*math.cos(a),radial*math.sin(a),z))
            return ids
        rings=[hex_ring(-.003),hex_ring(.003),ring(.004,.003),ring(.004,-.003)]
        for a,b in zip(rings,rings[1:]+rings[:1]):connect(a,b)
    else:
        previous=None
        tip=len(vertices)+1;vertices.append((0.,0.,-.02))
        for layer in range(1,145):
            z=-.02+.036*layer/144;indices=[]
            for k in range(48):
                angle=2*math.pi*k/48
                phase=(z/.00125-k/48)%1
                radius=.0036+.0008*(1-abs(2*phase-1))
                # 끝 6 mm만 점차 가늘게 만들어 나사산이 이어지는 뾰족한 끝을 표현한다.
                # 시각 메시만 바꾸며 충돌 원기둥·질량·흡착 높이는 변경하지 않는다.
                radius*=min(1.,(z+.02)/.006)
                indices.append(len(vertices)+1);vertices.append((radius*math.cos(angle),radius*math.sin(angle),z))
            if previous:connect(previous,indices)
            else:
                for k in range(len(indices)):
                    faces.append((tip,indices[(k+1)%len(indices)],indices[k]))
            previous=indices
        bottom=ring(.0065,.016,6,math.pi/6);top=ring(.0065,.02,6,math.pi/6)
        connect(bottom,top)
        for k in range(1,5):faces.extend([(top[0],top[k],top[k+1]),(bottom[0],bottom[k+1],bottom[k])])
    path.with_suffix('.mtl').write_text('newmtl Metal\nKa 0.22 0.22 0.22\nKd 0.58 0.58 0.58\nKs 0.45 0.45 0.45\nNs 96\nillum 2\n')
    path.write_text(f'mtllib {path.with_suffix(".mtl").name}\nusemtl Metal\n'+''.join('v %.9f %.9f %.9f\n'%v for v in vertices)+''.join('f %d %d %d\n'%f for f in faces))


def generate_profile(output, root, *, realistic=False, randomize=False, seed=0, speed=.5, count=10, compact=False, position_jitter_mm=0., acceleration=.5, screw_length_mm=40.):
    output=Path(output).resolve();root=Path(root).resolve();output.mkdir(parents=True,exist_ok=True)
    if not .1 <= speed <= .7:raise ValueError('Demo speed must be 0.1..0.7')
    if not math.isfinite(acceleration) or not 0 < acceleration <= 1:
        raise ValueError('Acceleration must be finite and in (0, 1]')
    if not 1 <= count <= 10:raise ValueError('Object count must be 1..10')
    if not math.isfinite(screw_length_mm) or not 10 <= screw_length_mm <= 40:
        raise ValueError('Screw length must be finite and in 10..40 mm')
    scene_packet(count,seed,position_jitter_mm)  # 파일 생성 전에 위치 변동 범위를 검증한다.
    rng=random.Random(seed)
    source=root/'src/ur5e_pick_place'
    prefix=output/'assets';share=prefix/'share/ur5e_pick_place';models=share/'models'
    models.mkdir(parents=True,exist_ok=True)
    index=prefix/'share/ament_index/resource_index/packages';index.mkdir(parents=True,exist_ok=True)
    (index/'ur5e_pick_place').touch()
    for folder in (source/'models').iterdir():
        if folder.is_dir():shutil.copytree(folder,models/folder.name,dirs_exist_ok=True)
    for kind in ['screw','nut']:
        path=models/kind/'model.sdf';tree=ET.parse(path);link=tree.getroot().find('model/link')
        # 원본 모델은 그대로 두고 프로필의 두 부품 모두 무채색 금속 재질로 맞춘다.
        for visual in link.findall('visual'):
            material=visual.find('material')
            if material is None:material=ET.SubElement(visual,'material')
            for tag,value in [('ambient','0.22 0.22 0.22 1'),('diffuse','0.58 0.58 0.58 1'),('specular','0.45 0.45 0.45 1')]:
                element=material.find(tag)
                if element is None:element=ET.SubElement(material,tag)
                element.text=value
        if realistic:
            # 첫 visual은 기존 형상 메타데이터용으로 남긴다. Gazebo에서는 투명하게 하고 메쉬를 표시한다.
            visual=link.find('visual');ET.SubElement(visual,'transparency').text='1'
            detail=copy.deepcopy(visual);detail.set('name','team2_detail');detail.remove(detail.find('transparency'))
            detail.remove(detail.find('geometry'));geom=ET.SubElement(detail,'geometry');mesh=ET.SubElement(geom,'mesh')
            mesh_path=models/kind/'detail.obj';write_mesh(mesh_path,kind)
            ET.SubElement(mesh,'uri').text=mesh_path.as_uri();link.append(detail);tree.write(path,encoding='unicode')
        else:
            tree.write(path,encoding='unicode')
    if screw_length_mm != 40.:
        path=models/'screw/model.sdf';tree=ET.parse(path);link=tree.getroot().find('model/link')
        length=screw_length_mm/1000.
        # 복사한 나사만 장축을 줄인다. 지름·높이·질량은 유지해 기존 흡착 조건을 약화하지 않는다.
        for cylinder in link.findall('collision/geometry/cylinder')+link.findall('visual/geometry/cylinder'):
            cylinder.find('length').text=str(length)
        for mesh in link.findall('visual/geometry/mesh'):
            ET.SubElement(mesh,'scale').text=f'1 1 {length/.04}'
        radius=float(link.findtext('collision/geometry/cylinder/radius'))
        mass=float(link.findtext('inertial/mass'))
        for axis in ['ixx','iyy']:
            link.find(f'inertial/inertia/{axis}').text=str(mass*(3*radius**2+length**2)/12)
        link.find('inertial/inertia/izz').text=str(mass*radius**2/2)
        tree.write(path,encoding='unicode')
    world=ET.parse(source/'worlds/workcell.sdf');w=world.getroot().find('world')
    # 큰 통도 Gazebo·MoveIt가 같은 SDF를 읽는다. 중심 좌표는 유지하고 내측 폭만 늘린다.
    for name in ['screw_bin','nut_bin']:
        for elem in w.find(f"model[@name='{name}']/link"):
            if elem.tag not in ['visual','collision']:continue
            box=elem.find('geometry/box/size')
            if box is None:continue
            size=[float(x) for x in box.text.split()];size=[.22 if abs(x-.15)<1e-8 else x for x in size];box.text=' '.join(map(str,size))
            pose=elem.find('pose')
            if pose is not None:
                values=[float(x) for x in pose.text.split()]
                values[:2]=[math.copysign(.1075,x) if abs(abs(x)-.0725)<1e-8 else x for x in values[:2]]
                pose.text=' '.join(map(str,values))
    camera=w.find("model[@name='top_camera']/pose");values=[float(x) for x in camera.text.split()];base_height=values[2]
    if randomize:
        values[2]=rng.uniform(.73,.79);camera.text=' '.join(map(str,values))
        value=rng.uniform(.28,.43)
        for material in w.findall("model[@name='ground_plane']//material"):
            for tag in ['ambient','diffuse']:material.find(tag).text=f'{value} {value} {value} 1'
        light=w.find('light');brightness=rng.uniform(.78,1.0)
        light.find('diffuse').text=f'{brightness} {brightness} {brightness} 1'
        light.find('direction').text=f'{rng.uniform(-.7,-.2)} {rng.uniform(-.4,.4)} -1'
        # 바탕색만 바꾸고 마커 흑백은 유지해 좌표 정답을 보존한다.
        texture=models/'work_mat/work_sheet.png';image=cv2.imread(str(texture));hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        color=(rng.randrange(25,55),rng.randrange(125,170),rng.randrange(20,50))
        image[hsv[:,:,1]>100]=color;cv2.imwrite(str(texture),image)
    mat=models/'work_mat/model.sdf';mat_tree=ET.parse(mat)
    mat_tree.getroot().find('.//albedo_map').text=(models/'work_mat/work_sheet.png').as_uri();mat_tree.write(mat,encoding='unicode')
    w.find("include[name='work_mat']/uri").text=(models/'work_mat').as_uri()
    world_path=output/'workcell.sdf';world.write(world_path,encoding='unicode')
    config=yaml.safe_load((source/'config/sim.yaml').read_text());p=config['pick_place_node']['ros__parameters']
    p.update(max_objects=count,world_file=str(world_path),max_velocity_scaling=float(speed),team2_acceleration_scaling=float(acceleration),scene_position_jitter_mm=float(position_jitter_mm),bin_drop_z=.10,settle_s=2.0,team2_world_bin_check=True,compact_cycle_return=compact)
    # 스폰·RViz 마커·집기 판정이 같은 프로필 모델 치수를 사용하게 한다.
    p['team2_models_dir']=str(models)
    config_path=output/'sim.yaml';config_path.write_text(yaml.safe_dump(config,sort_keys=False))
    calibration=output/'calib';calibration.mkdir(exist_ok=True)
    base=root/'data/calib/sim'
    shutil.copy2(base/'intrinsics.npz',calibration/'intrinsics.npz')
    with np.load(base/'intrinsics.npz') as data:K=data['camera_matrix']
    with np.load(base/'homography.npz') as data:H=data['H'];error=float(data['reproj_err_mm'])
    # 카메라 높이가 바뀌면 기존 H를 그대로 쓰지 않는다. 알려진 평면 투영으로 픽셀 축척을 갱신한다.
    ratio=(values[2]-.0005)/(base_height-.0005)
    transform=np.array([[ratio,0,K[0,2]*(1-ratio)],[0,ratio,K[1,2]*(1-ratio)],[0,0,1.]])
    np.savez(calibration/'homography.npz',H=H@transform,reproj_err_mm=error,undistorted=True)
    vision=yaml.safe_load((root/'src/vision_pick_place/config/params_sim.yaml').read_text())
    vp=vision['vision_node']['ros__parameters'];vp.update(intrinsics_path=str(calibration/'intrinsics.npz'),homography_path=str(calibration/'homography.npz'))
    vision_path=output/'vision.yaml';vision_path.write_text(yaml.safe_dump(vision,sort_keys=False))
    manifest=dict(sim_params=str(config_path),world_file=str(world_path),asset_prefix=str(prefix),
                  description_file=str(source/'urdf/team2_ten_slots.urdf.xacro'),vision_params=str(vision_path),
                  calibration_dir=str(calibration),max_objects=count,slot_capacity=10,compact_return=compact,bin_width_m=.22,camera_height_m=values[2],
                  speed=speed,acceleration=acceleration,seed=seed,position_jitter_mm=position_jitter_mm,realistic_visuals=realistic,randomized=randomize,screw_length_mm=screw_length_mm,
                  physical_contact_geometry=('unchanged primitive approximation' if screw_length_mm==40. else 'shorter screw primitive; original diameter, height and mass'),calibration_method='known camera height projection; validate using rendered ArUco')
    (output/'profile.json').write_text(json.dumps(manifest,indent=2));return manifest
