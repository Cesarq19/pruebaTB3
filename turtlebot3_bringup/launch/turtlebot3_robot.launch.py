import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

pkg_turtlebot_bringup = get_package_share_directory('turtlebot3_bringup')
pkg_turtlebot_control = get_package_share_directory('turtlebot3_control')
pkg_turtlebot_description = get_package_share_directory('turtlebot3_description')
pkg_ydlidar = get_package_share_directory('ydlidar_ros2_driver')
pkg_witmotion_imu = get_package_share_directory('witmotion_ros')

def generate_launch_description():
    # Declares launch arguments
    ydlidar_arg = DeclareLaunchArgument(
            'include_ydlidar',
            default_value='True',
            description='Indicates whether to include ydlidar launch.')
    ydlidar =  LaunchConfiguration('include_ydlidar')

    imu_arg = DeclareLaunchArgument(
            'include_imu',
            default_value='True',
            description='Indicates whether to include witmotion imu launch.')
    imu =  LaunchConfiguration('include_imu')

    # Includes turtlebot_description launch file
    include_turtlebot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot_bringup, 'launch', 'turtlebot_description.launch.py'),
        ),
        launch_arguments={
            'rsp': 'True',
        }.items()
    )

    # Include turtlebot_control launch file
    include_turtlebot_control =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot_control, 'launch', 'turtlebot3_control.launch.py'),
        ),
        launch_arguments={
        }.items()
    )

    # Include ydlidar launch file
    include_ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ydlidar, 'launch', 'ydlidar_launch.py'),
        ),
            condition=IfCondition(ydlidar)
    )

    # Include imu launch file
    include_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_witmotion_imu, 'launch', 'wt61c.launch.py'),
        ),
            condition=IfCondition(imu)
    )

    # Waits for turtlebot_description to set up robot_state_publisher.
    turtlebot_control_timer = TimerAction(period=5.0, actions=[include_turtlebot_control])
    # Defer sensors launch to avoid overhead while robot_state_publisher is setting up.
    ydlidar_timer = TimerAction(period=3.0, actions=[include_ydlidar])
    imu_timer = TimerAction(period=3.0, actions=[include_imu])

    return LaunchDescription([
        include_turtlebot_description,
        turtlebot_control_timer,
        imu_arg,
        imu_timer,
        ydlidar_arg,
        ydlidar_timer
    ])
