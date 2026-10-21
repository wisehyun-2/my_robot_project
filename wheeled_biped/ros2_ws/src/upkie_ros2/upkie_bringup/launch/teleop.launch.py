"""Keyboard teleoperation of Plan 1 through /cmd_vel (standard teleop_twist_keyboard).

    ros2 launch upkie_bringup teleop.launch.py [speed:=0.3] [turn:=0.6] [prefix:="xterm -e"]

teleop_twist_keyboard needs a terminal of its own, so it is started through `prefix` (xterm by
default, `sudo apt install xterm`).  Without xterm, run it directly instead:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.3 -p turn:=0.6
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('speed', default_value='0.3', description='initial linear speed [m/s]'),
        DeclareLaunchArgument('turn', default_value='0.6', description='initial yaw rate [rad/s]'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('prefix', default_value='xterm -e',
                              description='command prefix giving the node its own terminal'),
    ]

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix=LaunchConfiguration('prefix'),
        parameters=[{
            'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
            'turn': ParameterValue(LaunchConfiguration('turn'), value_type=float),
        }],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
    )

    return LaunchDescription(arguments + [teleop])
