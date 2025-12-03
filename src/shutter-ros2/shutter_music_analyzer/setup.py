from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'shutter_music_analyzer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/shutter_music_analyzer/launch', glob('launch/*')),
    ],
    install_requires=['setuptools', 'librosa', 'numpy'],
    zip_safe=True,
    maintainer='caleb.nieh@yale.edu',
    maintainer_email='calebnieh@gmail.com',
    description='Music analysis and oscillator-based motion control for Shutter robot',
    license='BSD',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'music_analyzer = shutter_music_analyzer.music_analyzer_node:main',
            'oscillator_control = shutter_music_analyzer.oscillator_node:main'
        ],
    },
)

