from setuptools import find_packages, setup

package_name = 'sancho_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'skeleton_detector = sancho_perception.skeleton_detector:main',
            'skeleton_3d_node = sancho_perception.skeleton_3d_node:main',
        ],
    },
)
