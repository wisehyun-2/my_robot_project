"""Display the DIABLO-lite URDF in RViz with sliders for the joints.

    ros2 launch diablo_lite_description display.launch.py [gui:=true|false]

gui:=true  -> joint_state_publisher_gui (sliders)
gui:=false -> joint_state_publisher (all joints at zero)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('diablo_lite_description')
    urdf_file = os.path.join(share_dir, 'urdf', 'diablo_lite.urdf')
    rviz_config_file = os.path.join(share_dir, 'rviz', 'diablo_lite.rviz')
    with open(urdf_file, encoding='utf-8') as f:
        robot_description = f.read()

    gui_arg = DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'],
                                    description='use joint_state_publisher_gui')
    show_gui = LaunchConfiguration('gui')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )
    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    # no odometry here: fix the view on base_link (the saved config uses odom for the simulation)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file, '-f', 'base_link'],
    )

    return LaunchDescription([
        gui_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])
