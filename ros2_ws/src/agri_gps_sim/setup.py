from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agri_gps_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Package manifest
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='farhan',
    maintainer_email='mohamedfarhankn@gmail.com',
    description='ENU GPS simulator for localization testing',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'enu_gps_sim_node = agri_gps_sim.enu_gps_sim_node:main',
        ],
    },
)
