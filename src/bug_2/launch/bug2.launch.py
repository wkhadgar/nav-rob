from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from datetime import datetime
import os

def generate_launch_description():
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

    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')

    bug2_node = ExecuteProcess(
        cmd=['ros2', 'run', 'bug_2', 'bug2main',
             '--goal_x', goal_x,
             '--goal_y', goal_y],
        output='screen'
    )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = f"bug2_{timestamp}"

    rosbag_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_name,
            '/cmd_vel',
            '/odom',
            '/scan',
            '/goal_marker'
        ],
        output='screen'
    )

    return LaunchDescription([
        goal_x_arg,
        goal_y_arg,
        bug2_node,
        rosbag_cmd
    ])
