from glob import glob
import os
from setuptools import setup

package_name = 'mir_launch_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MATCH',
    maintainer_email='match@ipa.fraunhofer.de',
    description='ROS 2 hardware bringup helpers for MiR robots',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'external_localization_broadcaster = mir_launch_hardware.external_localization_broadcaster:main',
            'mir_battery_state_publisher = mir_launch_hardware.mir_battery_state_publisher:main',
            'mir_pose_simple = mir_launch_hardware.mir_pose_simple:main',
            'mir_rosapi_audit = mir_launch_hardware.mir_rosapi_audit:main',
            'rgb_control = mir_launch_hardware.rgb_control:main',
        ],
    },
)
