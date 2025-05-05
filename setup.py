import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sonoptix_sonar'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ItsKalvik',
    maintainer_email='itskalvik@gmail.com',
    description='ROS 2 interface for Sonoptix sonars',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'echo = sonoptix_sonar.echo:main',
            'echo_imager = sonoptix_sonar.echo_imager:main'
        ],
    },
)
