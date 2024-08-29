import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lanealucys_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all config files.
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        # Include all rviz files.
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        # Include all urdf files.
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        # Include all world files.
        (os.path.join('share', package_name, 'world'), glob(os.path.join('world', '*.sdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lanealucy',
    maintainer_email='lanealucy@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'repeater = lanealucys_robot.repeater:main',
            'tf2_broadcaster = lanealucys_robot.tf2_broadcaster:main',
            'pose_rewriter = lanealucys_robot.pose_rewriter:main',
        ],
    },
)
