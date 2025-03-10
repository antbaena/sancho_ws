import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'py_human_face_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/models", glob(package_name + '/models/*')),
        ('share/' + package_name + "/fonts", glob(package_name + '/fonts/*')),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eulogioqt',
    maintainer_email='euquemada@gmail.com',
    description='Human face recognition',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'converter = py_human_face_recognition.yuvu_image_converter:main',
            'camera_parameters = py_human_face_recognition.camera_parameters:main',
            'camera = py_human_face_recognition.camera_capturer:main',
            'detector = py_human_face_recognition.human_face_detector:main',       
            'recognizer = py_human_face_recognition.human_face_recognizer:main',
            'logic = py_human_face_recognition.hri_logic_copy:main',
            'head = py_human_face_recognition.hri_head:main',
            'gui = py_human_face_recognition.hri_gui:main',
            'body = py_human_face_recognition.hri_body:main',
            'metrics = py_human_face_recognition.hri_metrics:main',
        ],
    },
)
