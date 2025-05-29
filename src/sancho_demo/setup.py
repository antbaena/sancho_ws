from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'sancho_demo'

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
    maintainer='antbaena',
    maintainer_email='antbaena@uma.es',
    description='TODO: Package description',
    license='GPL-3.0-or-later',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'head_demo = sancho_demo.head_demo:main',
        ],
    },
)
