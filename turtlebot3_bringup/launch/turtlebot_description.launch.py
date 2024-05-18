import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Arguments
    rsp_argument = DeclareLaunchArgument('rsp', default_value='true',
                          description='Run robot state publisher node.')

    namespace_argument = DeclareLaunchArgument('namespace', default_value='',
                            description='robot namespace')
    
    namespace = LaunchConfiguration('namespace')

    # Obtains turtlebot_description's share directory path.
    pkg_turtlebot_description = get_package_share_directory('turtlebot3_description')

    # Obtain urdf from xacro files.
    doc = xacro.process_file(os.path.join(pkg_turtlebot_description, 'urdf', 'turtlebot3_burger.urdf.xacro'))
    robot_desc = doc.toprettyxml(indent='  ')
    frame_prefix = [namespace, '/']
    params = {'robot_description': robot_desc,
              'publish_frequency': 30.0,
              'frame_prefix': frame_prefix}
    

    # Robot state publisher
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=namespace,
                output='both',
                parameters=[params],
                condition=IfCondition(LaunchConfiguration('rsp'))
    )

    return LaunchDescription([
        namespace_argument,
        namespace,
        rsp_argument,
        rsp,
    ])
