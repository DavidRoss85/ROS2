from setuptools import find_packages, setup

package_name = 'yolobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david-ross',
    maintainer_email='ross.d2@northeastern.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_basic = yolobot.yolo_basic:main',
            'yolo_viz = yolobot.yolo_viz:main',

        ],
    },
)
