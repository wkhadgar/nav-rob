from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node
from datetime import datetime
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('world_simulation')
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0')

    sdf_file = os.path.join(pkg_share, 'worlds', 'map.sdf')

    init_claim = ExecuteProcess(
        cmd=['echo', 'Iniciando...'],
        output='screen'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', sdf_file], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose')
        }.items()
    )
    
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory('world_simulation'),
        'rviz',
        'bug_world.rviz'
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    delay_actions_after_sdf = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=init_claim,
            on_exit=[
                gzserver_cmd,
                gzclient_cmd,
                spawn_robot,
                robot_state_publisher_cmd,
                start_rviz_cmd,
            ]
        )
    )

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models'))

    return LaunchDescription([
        x_pose_arg,
        y_pose_arg,
        set_env_vars_resources,

        init_claim,

        delay_actions_after_sdf,
    ])
