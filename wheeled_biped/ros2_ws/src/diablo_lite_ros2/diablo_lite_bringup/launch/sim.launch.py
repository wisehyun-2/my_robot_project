"""Plans 2 and 3 simulation: MCU node (MuJoCo + DIABLO-style MCU), cmd_vel bridge, RViz.

    ros2 launch diablo_lite_bringup sim.launch.py balancer:=lqi6|upkie_pid|upkie_mpc rviz:=true|false

balancer:=lqi6       Plan 2 (LQI6 law of the design report, torque-mode VMC legs)
balancer:=upkie_pid  Plan 3 (upkie wheel balancer PI inside the DIABLO MCU)
balancer:=upkie_mpc  Plan 3 (upkie MPC balancer inside the DIABLO MCU)
Drive it with /cmd_vel (bridge:=true, default) or with `ros2 run diablo_lite_ctrl teleop_node`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('diablo_lite_bringup')
    description_share = get_package_share_directory('diablo_lite_description')

    balancer = LaunchConfiguration('balancer')
    rviz = LaunchConfiguration('rviz')
    bridge = LaunchConfiguration('bridge')
    mcu_config = LaunchConfiguration('mcu_config')
    rviz_config = LaunchConfiguration('rviz_config')

    arguments = [
        DeclareLaunchArgument('balancer', default_value='lqi6', choices=['lqi6', 'upkie_pid', 'upkie_mpc'],
                              description='balance law run by the MCU'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'], description='start rviz2'),
        DeclareLaunchArgument('bridge', default_value='true', choices=['true', 'false'],
                              description='start the /cmd_vel -> /diablo/MotionCmd bridge (with heartbeat)'),
        DeclareLaunchArgument('mcu_config', default_value=os.path.join(bringup_share, 'config', 'mcu.yaml')),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(description_share, 'rviz', 'diablo_lite.rviz')),
    ]

    with open(os.path.join(description_share, 'urdf', 'diablo_lite.urdf'), encoding='utf-8') as f:
        robot_description = f.read()

    mcu_node = Node(
        package='diablo_lite_mcu',
        executable='mcu_node',
        name='mcu_node',
        output='screen',
        emulate_tty=True,
        parameters=[mcu_config, {'balancer': balancer}],
    )

    cmd_vel_bridge = Node(
        package='diablo_lite_ctrl',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
        condition=IfCondition(bridge),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', rviz_config],
    )

    return LaunchDescription(arguments + [
        mcu_node,
        cmd_vel_bridge,
        robot_state_publisher,
        rviz_node,
    ])
