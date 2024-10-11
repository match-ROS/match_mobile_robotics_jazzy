import os
from glob import glob
from setuptools import setup

package_name = 'mir_driver'

setup(
    name=package_name,
    version='1.1.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Martin Günther',
    maintainer_email='martin.guenther@dfki.de',
    description='A reverse ROS2 bridge for the MiR robot',
    license='BSD, Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mir_bridge = mir_driver.mir_bridge:main',
            'fake_mir_joint_publisher = mir_driver.fake_mir_joint_publisher:main',
            'time_synchronizer = mir_driver.time_synchronizer:main',
            'tf_remove_child_frames = mir_driver.tf_remove_child_frames:main',
        ],
    },
)
