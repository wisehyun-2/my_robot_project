from setuptools import find_packages, setup

package_name = 'upkie_spine'

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
    description='Plan 1 spine: MuJoCo simulation + upkie observers at 1 kHz behind a ROS 2 interface',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'spine_node = upkie_spine.spine_node:main',
        ],
    },
)
