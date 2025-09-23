from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from datetime import datetime
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('world_simulation')

    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='-1.0',
        description='Posição inicial X'
    )
    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='-1.0',
        description='Posição inicial Y'
    )
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='5.0',
        description='Coordenada X do objetivo'
    )
    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='0.0',
        description='Coordenada Y do objetivo'
    )

    sim_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose')
        }.items()
    )

    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')

    bug2_node = ExecuteProcess(
        cmd=['ros2', 'run', 'bug_2', 'bug2main',
             '--goal_x', goal_x,
             '--goal_y', goal_y],
        output='screen'
    )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    rosbag_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', f"bags/bug2_{timestamp}",
            '/cmd_vel',
            '/odom',
            '/scan',
            '/goal_marker'
        ],
        output='screen'
    )

    warn_load_cmd = ExecuteProcess(cmd=[
        'echo', 'Pressione enter para iniciar...',
    ], output='screen')

    load_cmd = ExecuteProcess(cmd=[
        'sleep', '50'
    ], output='screen')

    delay_actions_after_load = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_cmd,
            on_exit=[
                bug2_node,
                rosbag_cmd,
            ]
        )
    )

    return LaunchDescription([
        x_pose_arg,
        y_pose_arg,
        goal_x_arg,
        goal_y_arg,
        sim_world,
        warn_load_cmd,
        load_cmd,
        delay_actions_after_load,
    ])
