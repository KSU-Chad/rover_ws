import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')



    scout_share = get_package_share_directory('mentorpi_description')
    model_root = os.path.dirname(scout_share)   # .../install/scout_description/share

    # --- Path to ros_gz_sim's Gazebo launch file ---
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')

    # --- Build robot_description from xacro ---
    model_name = 'robot.urdf.xacro'
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('mentorpi_description'),
            'urdf',
            model_name
        ]),
    ])



    xacro_file = PathJoinSubstitution([
        FindPackageShare('my_rover'),
        'description',
        'my_rover.xacro',  # your xacro here
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file,
    ])

    robot_description = ParameterValue(robot_description_content, value_type=str)

    params = {
        'robot_description': robot_description,
        'use_sim_time': use_sim_time,
    }

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true',
        ),
        rsp,
        # --- Make Gazebo able to resolve model://scout_description/... ---
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=model_root + os.pathsep
        ),
        AppendEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=model_root + os.pathsep
        ),

    ])

