from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bluerov2_gui_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='loan',
    maintainer_email='loan@todo.todo',
    description='GUI teleoperation for BlueROV2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gui_teleop=bluerov2_gui_teleop.gui_teleop:main',
        ],
    },
)
