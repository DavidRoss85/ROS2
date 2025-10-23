import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'gps_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),['launch/gps_launch.py','launch/standalone_driver_launch.py','launch/rtk_driver_launch.py', 'launch/launch_and_record.py']), #Add this for incorporating launch into build
    ],
    install_requires=['setuptools', 'utm'],
    zip_safe=True,
    maintainer='david-ross',
    maintainer_email='ross.d2@northeastern.edu',
    description='Records data from a GPS device and publishes it',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_node = gps_driver.standalone_driver:main',
            'gps_record = gps_driver.listener:main',
            'rtk_node = gps_driver.rtk_driver:main',
            'rtk_record = gps_driver.listener2:main'
        ],
    },
)
