from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mobile_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'model'), glob('model/*')),
        (os.path.join('share', package_name, 'parameters'), glob('parameters/*')),
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OYASIMRAJA1316',
    maintainer_email='oyasimraja13raja@gmail.com',
    description='ROS 2 and Gazebo Harmonic simulation pipeline for a differential drive mobile robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_final = mobile_robot.controller_final:main',
            'controller2=mobile_robot.controller2:main',
        ],
    },
)