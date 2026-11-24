"""겹친 XY를 보존하는 실물 배치 재현용 충돌 필터. 적층 물리 모델은 아니다."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def slot_collision_model(source, name, directory):
    match=re.fullmatch(r'obj_([0-9])',name)
    if match is None:
        raise ValueError('충돌 필터는 obj_0..obj_9 슬롯에만 적용합니다.')
    tree=ET.parse(source)
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
