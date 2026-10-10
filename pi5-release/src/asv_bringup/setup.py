from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'asv_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhinav Gupta',
    maintainer_email='abbigupta.j18@gmail.com',
    description='Raspberry Pi 5 hardware launch package for the TAMU ASV.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={'console_scripts': []},
)
