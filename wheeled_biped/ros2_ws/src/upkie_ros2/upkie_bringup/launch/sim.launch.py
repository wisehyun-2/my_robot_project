"""Plan 1 simulation: spine (MuJoCo + observers), optional external balancer, RViz.

    ros2 launch upkie_bringup sim.launch.py balancer:=pid|mpc embedded:=true|false rviz:=true|false

embedded:=true  -> the spine runs the wheel balancer PI itself (balancer_node is not started;
                   balancer:=mpc is then ignored, a warning is printed)
embedded:=false -> spine + balancer_node (pid or mpc) talking over /upkie/observation and
                   /upkie/servo_commands
Velocity targets come from /cmd_vel (see teleop.launch.py).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory('upkie_bringup')
    description_share = get_package_share_directory('upkie_description')

    balancer = LaunchConfiguration('balancer')
    embedded = LaunchConfiguration('embedded')
    rviz = LaunchConfiguration('rviz')
    use_xacro = LaunchConfiguration('use_xacro')
    spine_config = LaunchConfiguration('spine_config')
    balancer_config = LaunchConfiguration('balancer_config')
    rviz_config = LaunchConfiguration('rviz_config')

    arguments = [
        DeclareLaunchArgument('balancer', default_value='pid', choices=['pid', 'mpc'],
                              description='external balancer type (only used when embedded:=false)'),
        DeclareLaunchArgument('embedded', default_value='true', choices=['true', 'false'],
                              description='run the wheel balancer PI inside the spine loop'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'],
                              description='start rviz2'),
        DeclareLaunchArgument('use_xacro', default_value='true', choices=['true', 'false'],
                              description='process upkie.xacro with xacro (false: use the pre-generated urdf/upkie.urdf)'),
        DeclareLaunchArgument('spine_config', default_value=os.path.join(bringup_share, 'config', 'spine.yaml')),
        DeclareLaunchArgument('balancer_config', default_value=os.path.join(bringup_share, 'config', 'balancer.yaml')),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(bringup_share, 'rviz', 'upkie.rviz')),
    ]

    # upkie.xacro includes its parts relative to the package root, hence the copy installed at the
    # share root by the upkie_description wrapper package (see its CMakeLists.txt).
    xacro_file = PathJoinSubstitution([FindPackageShare('upkie_description'), 'upkie.xacro'])
    robot_description_xacro = ParameterValue(Command([FindExecutable(name='xacro'), ' ', xacro_file]), value_type=str)
    with open(os.path.join(description_share, 'urdf', 'upkie.urdf'), encoding='utf-8') as f:
        robot_description_urdf = f.read()

    spine_node = Node(
        package='upkie_spine',
        executable='spine_node',
        name='spine_node',
        output='screen',
        emulate_tty=True,
        parameters=[spine_config, {
            'embedded_balancer': ParameterValue(embedded, value_type=bool),
            # the upkie MPC does not drive without a contact flag and the apparent-inertia observer
            # never triggers on a robot standing at rest (review A4/C2): ground-truth contact in simulation
            'use_sim_contact': ParameterValue(
                PythonExpression(["'", embedded, "' == 'false' and '", balancer, "' == 'mpc'"]), value_type=bool)}],
    )

    balancer_node = Node(
        package='upkie_balancer',
        executable='balancer_node',
        name='balancer_node',
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(embedded),
        parameters=[balancer_config, {'balancer': balancer}],
    )

    embedded_mpc_warning = LogInfo(
        condition=IfCondition(PythonExpression(["'", embedded, "' == 'true' and '", balancer, "' == 'mpc'"])),
        msg='embedded:=true runs the wheel balancer PI inside the spine; balancer:=mpc is ignored '
            '(use embedded:=false balancer:=mpc for the MPC balancer).')

    robot_state_publisher_xacro = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(use_xacro),
        parameters=[{'robot_description': robot_description_xacro}],
    )
    robot_state_publisher_urdf = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=UnlessCondition(use_xacro),
        parameters=[{'robot_description': robot_description_urdf}],
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
        embedded_mpc_warning,
        spine_node,
        balancer_node,
        robot_state_publisher_xacro,
        robot_state_publisher_urdf,
        rviz_node,
    ])
