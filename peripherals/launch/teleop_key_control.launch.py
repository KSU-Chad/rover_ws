import os
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # (declare parameter)
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=os.environ['HOST'],
#        default_value=os.environ['HOST', 'mentorpi'],
        description='Name of the robot'
    )

    # (set ROS naming space)
    push_namespace = PushRosNamespace(
        namespace=LaunchConfiguration('robot_name')
    )

    # (teleop_key_control node)
    teleop_key_control_node = Node(
        package='peripherals',
        executable='teleop_key_control',
        name='teleop_key_control',
        output='screen',
        prefix='xterm -e'
    )

    # (create launch description)
    ld = LaunchDescription()

    # (add parameter declaring and ROS naming space setting)
    ld.add_action(robot_name_arg)
    ld.add_action(push_namespace)

    # (add a node)
    ld.add_action(teleop_key_control_node)

    return ld

if __name__ == '__main__':
    # (create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
