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
    maintainer='Abhinav Gupta',
    maintainer_email='abbigupta.j18@gmail.com',
    description='Pi-side command arbitration and MAVROS adapter for the TAMU ASV.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'keyboard_teleop = asv_control.keyboard_teleop:main',
            'cmd_vel_mux = asv_control.cmd_vel_mux:main',
            'mavros_velocity_adapter = '
            'asv_control.mavros_velocity_adapter:main',
        ],
    },
)
