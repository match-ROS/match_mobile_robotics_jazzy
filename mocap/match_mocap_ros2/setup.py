from setuptools import setup

package_name = 'match_mocap_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='MATCH',
    maintainer_email='match@ipa.fraunhofer.de',
    description='Qualisys QTM to ROS 2 bridge over SSH',
    license='BSD',
    entry_points={
        'console_scripts': [
            'qualisys_ssh_bridge = match_mocap_ros2.qualisys_ssh_bridge:main',
        ],
    },
)
