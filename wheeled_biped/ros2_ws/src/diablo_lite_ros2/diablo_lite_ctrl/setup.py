from setuptools import find_packages, setup

package_name = 'diablo_lite_ctrl'

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
    maintainer='mjeong',
    maintainer_email='mjeong497@gmail.com',
    description='Host-side control of the DIABLO-lite robot: keyboard teleop and cmd_vel bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_node = diablo_lite_ctrl.teleop_node:main',
            'cmd_vel_bridge = diablo_lite_ctrl.cmd_vel_bridge:main',
        ],
    },
)
