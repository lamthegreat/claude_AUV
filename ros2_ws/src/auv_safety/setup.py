from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'auv_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AUV Developer',
    maintainer_email='jnlamoureux@gmail.com',
    description='Safety monitor for the AUV.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_monitor_node = auv_safety.safety_monitor_node:main',
        ],
    },
)
