from setuptools import setup
import os
from glob import glob

package_name = 'agri_fake_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Farhan',
    maintainer_email='mohamedfarhankn@gmail.com',
    description='Fake localization for agricultural UGV testing',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fake_localization_node = agri_fake_localization.fake_localization_node:main',
        ],
    },
)

