from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'auv_sensor_hub'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AUV Developer',
    maintainer_email='jnlamoureux@gmail.com',
    description='Pi-side bridge node for Teensy 4.1 sensor hub.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_hub_node = auv_sensor_hub.sensor_hub_node:main',
        ],
    },
)
