"""팀원 2의 나사별 길이 모델. 원본 SDF와 비전 클래스 이름은 변경하지 않는다."""
from dataclasses import replace
import math
import json
import os
from pathlib import Path
import re
import tempfile
import xml.etree.ElementTree as ET


MIN_LENGTH_MM, MAX_LENGTH_MM = 10, 60
_NAME = re.compile(r'screw__length_([0-9]{3})mm\Z')


def length_key(length_mm):
    if isinstance(length_mm,bool) or not isinstance(length_mm,(float,int)) or not math.isfinite(length_mm):
        raise ValueError('나사 length_mm는 유한한 숫자여야 합니다.')
    if not MIN_LENGTH_MM <= length_mm <= MAX_LENGTH_MM:
        raise ValueError(f'나사 length_mm는 {MIN_LENGTH_MM}..{MAX_LENGTH_MM} 범위여야 합니다.')
    return int(math.floor(length_mm+.5))


def length_from_key(name):
    match=_NAME.fullmatch(name)
    return length_key(int(match[1])) if match else None


def ensure_length_model(models_dir, length_mm):
    """같은 몸통 지름·높이·질량의 개별 길이 모델을 안전한 이름으로 캐시한다."""
    length=length_key(length_mm)
    root=Path(models_dir).resolve()
    manifest=root.parents[3]/'profile.json' if len(root.parents)>3 else None
    if manifest is None or not manifest.is_file():
        raise ValueError('나사별 길이는 생성한 팀원 2 프로필의 models 폴더를 사용해야 합니다.')
    profile=json.loads(manifest.read_text())
    if (Path(profile['asset_prefix'])/'share/ur5e_pick_place/models').resolve()!=root:
        raise ValueError('모델 폴더와 프로필 경로가 다릅니다.')
    key=f'screw__length_{length:03d}mm'
    source=root/'screw/model.sdf'
    tree=ET.parse(source);model=tree.getroot().find('model');link=model.find('link')
    old=float(link.findtext('collision/geometry/cylinder/length'))
    new=length/1000.
    if not math.isfinite(old) or old <= 0:
        raise ValueError('기준 나사 길이가 올바르지 않습니다.')
    model.set('name',key)
    for cylinder in link.findall('collision/geometry/cylinder')+link.findall('visual/geometry/cylinder'):
        cylinder.find('length').text=str(new)
    # 20 mm 기준 프로필도 지원한다. 기존 mesh 축척에 길이 비율을 곱한다.
    for mesh in link.findall('visual/geometry/mesh'):
        scale=mesh.find('scale')
        values=list(map(float,scale.text.split())) if scale is not None else [1.,1.,1.]
        if scale is None:scale=ET.SubElement(mesh,'scale')
        scale.text=f'{values[0]:g} {values[1]:g} {values[2]*new/old:g}'
    radius=float(link.findtext('collision/geometry/cylinder/radius'));mass=float(link.findtext('inertial/mass'))
    for axis in ['ixx','iyy']:
        link.find(f'inertial/inertia/{axis}').text=str(mass*(3*radius**2+new**2)/12)
    link.find('inertial/inertia/izz').text=str(mass*radius**2/2)
    target=Path(models_dir)/key/'model.sdf';content=ET.tostring(tree.getroot(),encoding='unicode')
    if not target.exists() or target.read_text()!=content:
        target.parent.mkdir(parents=True,exist_ok=True)
        # 스폰과 집기 노드가 동시에 요청해도 중간까지 쓰인 SDF를 읽지 않게 한다.
        with tempfile.NamedTemporaryFile('w',dir=target.parent,delete=False) as stream:
            temporary=stream.name;stream.write(content)
        try:os.replace(temporary,target)
        finally:Path(temporary).unlink(missing_ok=True)
    return key,target


def apply_packet_sizes(requests, packet, models_dir):
    """길이가 없는 기존 패킷은 기본 모델을 유지하며 원본 패킷을 수정하지 않는다."""
    by_id={obj.get('id'):obj for obj in packet.get('objects',[])}
    result=[]
    for request in requests:
        obj=by_id.get(request.source_id,{})
        length=obj.get('length_mm')
        if request.part_type=='screw' and 'length_status' in obj:
            # 측정 실패를 기본 40 mm로 조용히 대체하면 다시 겹침이 생길 수 있다.
            # 새 측정 모드에서는 안정화된 길이가 준비될 때까지 기존 장면을 보존한다.
            if obj['length_status']!='stable' or length is None:
                raise ValueError(f"id={request.source_id}: 나사 길이 측정이 준비되지 않았습니다 "
                                 f"({obj['length_status']}). 3프레임 안정화·보정·가림 여부를 확인하세요.")
        if request.part_type=='screw' and length is not None:
            key,_=ensure_length_model(models_dir,length)
            request=replace(request,part_type=key)
        result.append(request)
    return result
