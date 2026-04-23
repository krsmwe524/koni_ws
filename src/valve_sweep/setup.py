from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'valve_sweep'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kklab',
    maintainer_email='kklab-common@kklab.com',
    description='MPYE servo valve neutral voltage sweep',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'sweep_node = valve_sweep.sweep_node:main',
        ],
    },
)
