from setuptools import setup
import os
from glob import glob

package_name = 'agri_fake_localization'

setup(
    name=package_name,
    version='0.0.1',
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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='farhan',
    maintainer_email='mohamedfarhankn@gmail.com',
    description='Fake localization using perfect kinematic motion model',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'fake_localization_node = agri_fake_localization.fake_localization_node:main',
        ],
    },
)
