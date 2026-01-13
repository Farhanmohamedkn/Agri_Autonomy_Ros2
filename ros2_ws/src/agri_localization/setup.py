from setuptools import setup
import os
from glob import glob

package_name = 'agri_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='farhan',
    maintainer_email='mohamedfarhankn@gmail.com',
    description='Localization stack (EKF + helpers)',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'pose_cov_wrapper_node = agri_localization.pose_cov_wrapper_node:main',
        ],
    },
)
