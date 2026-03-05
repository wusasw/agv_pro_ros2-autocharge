import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rviz_config_dir = os.path.join(
        get_package_share_directory('agv_pro_gazebo'),
        'rviz',
        'agvpro_display.rviz')

    urdf_file = os.path.join(
        get_package_share_directory('agv_pro_gazebo'),
        'urdf',
        'agv_pro.urdf'
    )

    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()

    return LaunchDescription([

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            condition=IfCondition(use_rviz),
            output='screen')

    ])
