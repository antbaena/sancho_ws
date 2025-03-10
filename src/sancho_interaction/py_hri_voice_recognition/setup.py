import os
from glob import glob
from setuptools import find_packages,setup

package_name = 'py_hri_voice_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/sounds", glob(package_name + '/sounds/*')),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mapir',
    maintainer_email='famoreno@uma.es',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'microphone_capturer= py_hri_voice_recognition.microphone_publisher:main',
            'microphone_test= py_hri_voice_recognition.microphone_test:main',
            'audio_direction= py_hri_voice_recognition.audio_direction:main',
            'test= py_hri_voice_recognition.microphone_test:main',
            'assistant= py_hri_voice_recognition.assistant:main',
            'assistant_llm= py_hri_voice_recognition.assistant_llm:main',
            'helper= py_hri_voice_recognition.assistant_helper:main',
            'whisper_stt= py_hri_voice_recognition.whisper_stt:main',
            'google_stt= py_hri_voice_recognition.google_stt:main',
            'hri_tts= py_hri_voice_recognition.hri_tts:main',
            'python_tts= py_hri_voice_recognition.python_tts:main',
            'voice_detection= py_hri_voice_recognition.voice_detection:main',
            'speaker_controller= py_hri_voice_recognition.speaker_controller:main',
        ],
    },
)
