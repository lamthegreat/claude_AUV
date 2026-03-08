from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'auv_state_estimator'

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
    description='Sensor fusion / state estimator node.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_estimator_node = auv_state_estimator.state_estimator_node:main',
        ],
    },
)
