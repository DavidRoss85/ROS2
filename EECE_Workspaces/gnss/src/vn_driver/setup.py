import os
from setuptools import find_packages, setup

package_name = 'vn_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),['launch/driver_launch.py','launch/launch_and_record.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david-ross',
    maintainer_email='ross.d2@northeastern.edu',
    description='Publishes data from a Vectornav IMU',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vn_node = vn_driver.imu_driver:main'
        ],
    },
)
