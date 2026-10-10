from setuptools import find_packages, setup


package_name = 'asv_sensors'

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
    maintainer='Abhinav Gupta',
    maintainer_email='abbigupta.j18@gmail.com',
    description='Physical sensor drivers for the TAMU ASV.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'tfmini_driver = asv_sensors.tfmini_driver:main',
        'ds18b20_driver = asv_sensors.ds18b20_driver:main',
        'bme280_driver = asv_sensors.bme280_driver:main',
    ]},
)
