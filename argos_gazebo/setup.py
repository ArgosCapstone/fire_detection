from setuptools import setup
import os
from glob import glob

package_name = 'argos_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eugene',
    maintainer_email='eugene@sju.ac.kr',
    description='Fire detection simulation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'x1_node = argos_gazebo.x1_node:main',
            'quadrotor_node = argos_gazebo.quadrotor_node:main',
        ],
    },
)