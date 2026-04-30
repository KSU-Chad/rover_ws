import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # Package paths
    controller_pkg = get_package_share_directory('controller')
    my_rover_pkg = get_package_share_directory('my_rover')

    # Process xacro directly — bypasses the localuser hardcoded path bug
    # use_ros2_control=false skips the gz plugin block (not needed on real robot)
    xacro_file = os.path.join(my_rover_pkg, 'description', 'my_rover.xacro')
    robot_description = xacro.process_file(
        xacro_file,
        mappings={'use_ros2_control': 'false'}
    ).toxml()

    # --- Node 1: Hardware driver (talks to the physical board via /dev/rrc) ---
    ros_robot_controller_node = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        output='screen',
    )

    # --- Node 2: Robot state publisher (publishes URDF + TF tree) ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    )

    # --- Node 3: Odom publisher (mecanum kinematics + motor commands) ---
    # Inlined directly — odom_publisher.launch.py has hardcoded localuser paths
    # and double-launches ros_robot_controller, so we bypass it entirely.
    # Delayed 2s to let the hardware driver finish initializing.
    calibrate_params = os.path.join(controller_pkg, 'config', 'calibrate_params.yaml')

    odom_publisher_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller',
                executable='odom_publisher',
                name='odom_publisher',
                output='screen',
                parameters=[calibrate_params, {
                    'base_frame_id': 'base_footprint',
                    'odom_frame_id': 'odom',
                    'pub_odom_topic': True,
                }],
            )

        ]
    )
    

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    return LaunchDescription([
        ros_robot_controller_node,
        robot_state_publisher_node,
        odom_publisher_node,
        joint_state_publisher_node,
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
