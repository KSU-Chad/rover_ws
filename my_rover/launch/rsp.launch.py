import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.conditions import IfCondition, UnlessCondition

import xacro

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')
    frame_prefix = LaunchConfiguration('frame_prefix')

    # Process the URDF file
    pkg_path = get_package_share_directory('mentorpi_description')
    xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    # frame_prefix = LaunchConfiguration('frame_prefix', default='')
    
    # Create a robot_state_publisher node
    params = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time,
        'frame_prefix': frame_prefix
        }
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # joint_state_publisher_gui
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        # condition=IfCondition(use_gui),
    )  
    

# Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use joint_state_publisher_gui if true, otherwise joint_state_publisher',
        ),
        DeclareLaunchArgument(
            'frame_prefix',
            default_value='',
            description='TF frame prefix (useful for multi-robot)',
        ),
 
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        # node_robot_state_publisher,
    ])
