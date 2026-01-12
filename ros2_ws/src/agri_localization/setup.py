from setuptools import setup
import os
from glob import glob

package_name = 'agri_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Package manifest
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),

        # Install config files
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='farhan',
    maintainer_email='mohamedfarhankn@gmail.com',
    description='Localization stack (EKF) for agricultural UGV',
    license='Apache License 2.0',

    # 🔴 THIS PART IS REQUIRED
    entry_points={
        'console_scripts': [
            'pose_cov_wrapper_node = agri_localization.pose_cov_wrapper_node:main',
        ],
    },
)
