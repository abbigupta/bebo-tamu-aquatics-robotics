from setuptools import find_packages, setup


package_name = 'asv_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abbi',
    maintainer_email='abbi@todo.todo',
    description='Waypoint guidance for the TLU ASV.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'waypoint_follower = asv_autonomy.waypoint_follower:main',
        'square_mission = asv_autonomy.square_mission:main',
    ]},
)
