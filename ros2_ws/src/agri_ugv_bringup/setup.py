from setuptools import setup
import os
from glob import glob

package_name = 'agri_ugv_bringup'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Farhan',
    maintainer_email='mohamedfarhankn@gmail.com',
    description='Bringup launch for agricultural UGV (localization + mission + control)',
    license='MIT',
)
