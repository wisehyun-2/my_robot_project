from setuptools import find_packages, setup

package_name = 'upkie_balancer'

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
    description='Plan 1 balancer: upkie wheel balancer PI / MPC on the spine observation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'balancer_node = upkie_balancer.balancer_node:main',
        ],
    },
)
