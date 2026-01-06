from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agri_ugv_bringup'

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

        # Config / params (future-proof)
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'params'),
            glob('params/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Farhan',
    maintainer_email='mohamedfarhankn@gmail.com',
    description='Bringup package for agricultural UGV autonomy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
