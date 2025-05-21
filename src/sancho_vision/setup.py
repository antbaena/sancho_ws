from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'sancho_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mapir',
    maintainer_email='antoniocanetebaena1234@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_detector_node = sancho_vision.face_detector_node:main',
            'face_detector_lifecycle_node = sancho_vision.face_detector_lifecycle_node:main',
            'face_tracker_node = sancho_vision.face_tracker_node:main',
            'face_tracker_lifecycle_node = sancho_vision.face_tracker_lifecycle_node:main',
            
        ],
    },
)
