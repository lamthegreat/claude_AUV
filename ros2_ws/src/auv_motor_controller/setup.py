from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'auv_motor_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config', 'thruster_configs'),
            glob(os.path.join('config', 'thruster_configs', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AUV Developer',
    maintainer_email='jnlamoureux@gmail.com',
    description='Pi-side bridge and thruster allocator for Teensy 4.0 motor controller.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller_node = auv_motor_controller.motor_controller_node:main',
        ],
    },
)
