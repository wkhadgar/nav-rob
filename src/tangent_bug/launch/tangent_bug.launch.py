from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
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

    tangent_bug = ExecuteProcess(
        cmd=['ros2', 'run', 'tangent_bug', 'tangent_bug',
             '--goal_x', LaunchConfiguration('goal_x'),
             '--goal_y', LaunchConfiguration('goal_y')],
        output='screen'
    )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    rosbag_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', f"bags/tangent_bug_{timestamp}",
            '/cmd_vel',
            '/odom',
            '/bug_state',
            '/scan',
            '/goal_marker'
        ],
        output='screen'
    )

    return LaunchDescription([
        x_pose_arg,
        y_pose_arg,
        goal_x_arg,
        goal_y_arg,
        sim_world,
        tangent_bug,
        rosbag_cmd
    ])
