import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ur5e_pick_place'


def share(*parts):
    return os.path.join('share', package_name, *parts)


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share(), ['package.xml']),
        (share('launch'), glob('launch/*.launch.py')),
        (share('config'), glob('config/*.yaml')),
        (share('urdf'), glob('urdf/*.xacro')),
        (share('worlds'), glob('worlds/*.sdf')),
    ] + [
        # models/<part_type>/ 폴더를 통째로 설치. 공구 종류를 늘려도 여기는 안 고친다 (폴더 추가 후 colcon build 만)
        (share('models', os.path.basename(d)), glob(os.path.join(d, '*'))) for d in sorted(glob('models/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ODK2',
    maintainer_email='dhehehrud@naver.com',
    description='Gazebo UR5e pick-and-place driven by the vision pipeline',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'pick_place_node = ur5e_pick_place.pick_place_node:main',
        ],
    },
)
