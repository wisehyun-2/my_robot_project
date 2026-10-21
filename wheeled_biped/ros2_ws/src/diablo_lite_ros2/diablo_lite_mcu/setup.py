from setuptools import find_packages, setup

package_name = 'diablo_lite_mcu'

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
    description='DIABLO-lite MCU node (Plans 2 and 3): MuJoCo + DIABLO-style MCU behind the DIABLO ROS 2 interface',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mcu_node = diablo_lite_mcu.mcu_node:main',
        ],
    },
)
