import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -e',  # 或 'gnome-terminal --' 替换为你的终端命令
            remappings=[
                ('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')
            ]
        )
    ])
