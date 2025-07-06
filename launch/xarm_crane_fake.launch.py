from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    dof = LaunchConfiguration('dof', default=6)
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    robot_type = LaunchConfiguration('robot_type', default='xarm')

    # robot moveit servo launch
    # xarm_moveit_servo/launch/_robot_moveit_servo.launch.py
    robot_moveit_servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_crane_game'), 'launch', '_xarm_crane_fake.launch.py'])),
        launch_arguments={
            'dof': dof,
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'robot_type': robot_type,
        }.items(),
    )

    config_xarm_crane_game = os.path.join(
        get_package_share_directory('xarm_crane_game'),
        'config',
        'xarm_crane_game.yaml'
    )
    
    delayd_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='xarm_crane_game',
                executable='joy_to_servo_node',
                name='joy_to_servo_node',
                parameters=[config_xarm_crane_game]
            )
        ]
    )

    return LaunchDescription([
        robot_moveit_servo_launch,
        delayd_node
    ])