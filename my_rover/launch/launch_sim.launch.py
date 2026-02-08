import os
import launch
import launch_ros

from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

from launch.actions import LogInfo

from launch.actions import TimerAction


def generate_launch_description():
#    description_package_name="scout_description"
    description_package_name="my_rover"

    controllers_yaml = os.path.join(
        get_package_share_directory('my_rover'),
        'config',
        'my_controllers.yaml'
    )

    set_ctrl_yaml = SetEnvironmentVariable(
        name='MY_ROVER_CONTROLLERS',
        value=controllers_yaml
    )


    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
#            get_package_sfrom launch.actions import SetEnvironmentVariablehare_directory(description_package_name),'launch', 'scout_base_description.launch.py'
            get_package_share_directory(description_package_name),'launch', 'my_rover_description.launch.py'
            )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value="test_track.sdf",
        description='World to load'
        # -- /opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/empty.sdf ---
    )

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

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )


    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        # arguments=['-topic', 'robot_description',
                        arguments=['-file', '/home/localuser/mentorpi_ws/src/driver/mentorpi_description/sdf/robot.sdf',
                                   '-name', 'my_rover',
                                   '-z', '0.22'],
                                #    '-y', '1.5708'],
                        output='screen')


    # Launch the ROS-Gazebo bridge for normal topics
    control_package_name="my_rover"
    bridge_params = os.path.join(get_package_share_directory(control_package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_cont",
            '--controller-ros-args',
            '-r /diff_cont/cmd_vel:=/cmd_vel'
        ],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_spawners = TimerAction(
    period=6.0,
    actions=[joint_broad_spawner, diff_drive_spawner]
)

    log_gazebo = LogInfo(
    msg=['****** Starting Gazebo with world: ', world]
    )
    log_spawn = LogInfo(
    msg='****** Spawning robot entity into Gazebo'
    )
    log_bridge = LogInfo(
    msg=['****** Starting ros_gz_bridge using config: ', bridge_params]
    )
    log_diff_drive = LogInfo(
    msg='****** Spawning diff_drive controller'
    )
    log_joint_broad = LogInfo(
    msg='****** Spawning joint_state_broadcaster'
    )

    # Launch them all!
    return LaunchDescription([
        set_ctrl_yaml,
        rsp,
        world_arg,
        log_gazebo,
        gazebo,
        log_spawn,
        spawn_entity,
        log_bridge,
        ros_gz_bridge,
        delayed_spawners,
        # log_diff_drive,
        # diff_drive_spawner,
        # log_joint_broad,
        # joint_broad_spawner
    ])



