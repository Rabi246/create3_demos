import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'create3_demos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samanthav',
    maintainer_email='samanthav@todo.todo',
    description='Create 3 demo nodes for learning ROS 2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bump_and_go = create3_demos.bump_and_go:main',
            'drive_square = create3_demos.drive_square:main',
            'action_mission = create3_demos.action_mission:main',
        ],
    },
)
