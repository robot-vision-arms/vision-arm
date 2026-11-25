"""겹친 XY를 보존하는 실물 배치 재현용 충돌 필터. 적층 물리 모델은 아니다."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def slot_collision_model(source, name, directory):
    match=re.fullmatch(r'obj_([0-9])',name)
    if match is None:
        raise ValueError('충돌 필터는 obj_0..obj_9 슬롯에만 적용합니다.')
    tree=ET.parse(source)
    # 이전 프로필에도 적용한다. 디스크의 원본 모델은 보존하고 Gazebo에 보내는
    # 복사본의 모든 visual에 같은 회색을 명시한다(길이별 파생 모델 포함).
    for visual in tree.getroot().findall('.//visual'):
        material=visual.find('material')
        if material is None:material=ET.SubElement(visual,'material')
        for tag,color in [('ambient','0.22 0.22 0.22 1'),
                          ('diffuse','0.58 0.58 0.58 1'),
                          ('specular','0.45 0.45 0.45 1')]:
            element=material.find(tag)
            if element is None:element=ET.SubElement(material,tag)
            element.text=color
    # 각 부품에는 서로 겹치지 않는 비트 하나를 준다. 부품끼리의 AND는 0,
    # 기본 0xffff인 바닥·상자·로봇과의 AND는 0이 아니므로 접촉이 유지된다.
    mask=1 << int(match[1])
    collisions=tree.getroot().findall('.//collision')
    if not collisions:raise ValueError('부품 충돌 형상이 없습니다.')
    for collision in collisions:
        surface=collision.find('surface')
        if surface is None:surface=ET.SubElement(collision,'surface')
        contact=surface.find('contact')
        if contact is None:contact=ET.SubElement(surface,'contact')
        for tag in ['collide_bitmask','category_bitmask']:
            value=contact.find(tag)
            if value is None:value=ET.SubElement(contact,tag)
            value.text=str(mask)
    path=Path(directory)/name/'model.sdf';path.parent.mkdir(parents=True,exist_ok=True)
    tree.write(path,encoding='unicode')
    return path
