import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vision_pick_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ODK2',
    maintainer_email='dhehehrud@naver.com',
    description='Robot pick-and-place vision pipeline (X, Y, angle estimation)',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'vision_node = vision_pick_place.vision_node:main',
            'robot_bridge = vision_pick_place.robot_bridge:main',
            'robot_server = vision_pick_place.robot_server:main',
            'gui = vision_pick_place.gui.main_window:main',
        ],
    },
)
