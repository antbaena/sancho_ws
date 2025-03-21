from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'sancho_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
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
            'movenet_inference_node = sancho_perception.movenet_inference_node:main',
            'movenet_postprocessing_node = sancho_perception.movenet_postprocessing_node:main',
            'group_detection_node = sancho_perception.group_detection_node:main',
            'group_navigation_node = sancho_perception.group_navigation_node:main',
        ],
    },
)
