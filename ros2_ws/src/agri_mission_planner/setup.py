from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agri_mission_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Package manifest
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # Parameter files
        (os.path.join('share', package_name, 'params'),
            glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Farhan',
    maintainer_email='mohamedfarhankn@gmail.com',
    description='Mission-level coverage planning for agricultural UGVs',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_node = agri_mission_planner.mission_node:main',
        ],
    },
)
