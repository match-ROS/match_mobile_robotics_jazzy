from glob import glob
import os
from setuptools import setup

package_name = 'mur_mir_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*.service')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MATCH',
    maintainer_email='match@ipa.fraunhofer.de',
    description='Standalone guarded DS4 teleoperation for the mur620d MiR base.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ds4_mir_teleop = mur_mir_teleop.ds4_mir_teleop:main',
            'standalone_supervisor = mur_mir_teleop.standalone_supervisor:main',
        ],
    },
)
