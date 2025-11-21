from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'music_analyzer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/music_analyzer/launch', ['launch/music_dance.launch.py']),
        ('share/music_analyzer/launch', glob('launch/*')),
    ],
    install_requires=['setuptools', 'librosa', 'numpy'],
    zip_safe=True,
    maintainer='caleb.nieh@yale.edu',
    maintainer_email='calebnieh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'music_analyzer = music_analyzer.music_analyzer_node:main'
        ],
    },
)
