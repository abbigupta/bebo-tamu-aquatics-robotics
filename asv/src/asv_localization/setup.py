from glob import glob

from setuptools import find_packages, setup


package_name = 'asv_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abbi',
    maintainer_email='abbi@todo.todo',
    description='ASV localization nodes and configuration.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'ground_truth_localizer = asv_localization.ground_truth_localizer:main',
    ]},
)
