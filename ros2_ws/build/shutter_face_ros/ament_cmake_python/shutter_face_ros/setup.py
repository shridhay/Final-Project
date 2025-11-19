from setuptools import find_packages
from setuptools import setup

setup(
    name='shutter_face_ros',
    version='0.0.1',
    packages=find_packages(
        include=('shutter_face_ros', 'shutter_face_ros.*')),
)
