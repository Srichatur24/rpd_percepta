from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rpd_percepta'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chatur',
    maintainer_email='srichatur24@kmit.edu.in',
    description='ROS perception package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_object_detector = rpd_percepta.yolo_object_detector:main',
            'yolo_convert_to_3d = rpd_percepta.yolo_convert_to_3d:main'
        ],
    },
)
