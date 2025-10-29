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
        default_value='4.0',
        description='Posição inicial X'
    )
    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Posição inicial Y'
    )
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='-4.0',
        description='Coordenada X do objetivo'
    )
    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='-4.0',
        description='Coordenada Y do objetivo'
    )

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')

    sim_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose')
        }.items()
    )



    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    rosbag_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', f"bags/rrt_{timestamp}",
            '/cmd_vel',
            '/odom',
            '/goal_marker'
        ],
        output='screen'
    )

    calculate_rrt_cmd = ExecuteProcess(cmd=[
        'ros2', 'run', 'map_based', 'calculate_rrt',
    ], output='screen')

    control_node = ExecuteProcess(
        cmd=['ros2', 'run', 'map_based', 'map_based_main',
             '--x_pose', x_pose,
             '--y_pose', y_pose,
             '--goal_x', goal_x,
             '--goal_y', goal_y,
             ],
        output='screen'
    )

    load_cmd = ExecuteProcess(cmd=[
        'sleep', '5'
    ], output='screen')

    delay_actions_after_load = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_cmd,
            on_exit=[
                control_node,
                rosbag_cmd,
            ]
        )
    )

    return LaunchDescription([
        x_pose_arg,
        y_pose_arg,
        goal_x_arg,
        goal_y_arg,
        calculate_rrt_cmd,
        sim_world,
        load_cmd,
        delay_actions_after_load,
    ])
