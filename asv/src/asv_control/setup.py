from setuptools import find_packages, setup


package_name = 'asv_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abbi',
    maintainer_email='abbi@todo.todo',
    description='Twin-thruster command and safety nodes for the TLU ASV.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'twin_thruster_controller = '
            'asv_control.twin_thruster_controller:main',
            'thruster_test = asv_control.thruster_test:main',
            'keyboard_teleop = asv_control.keyboard_teleop:main',
            'cmd_vel_mux = asv_control.cmd_vel_mux:main',
            'velocity_controller = asv_control.velocity_controller:main',
            'ocean_current = asv_control.ocean_current:main',
            'mavros_velocity_adapter = '
            'asv_control.mavros_velocity_adapter:main',
        ],
    },
)
