from setuptools import setup

package_name = 'match_mocap_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosmatch',
    maintainer_email='rosmatch@example.com',
    description='MuR base GUI extension for Qualisys Mocap',
    license='BSD',
    entry_points={
        'console_scripts': [
            'mocap_gui = match_mocap_gui.mocap_gui:main',
        ],
    },
)
