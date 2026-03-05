import os
from launch import LaunchDescription
from launch_ros.actions import Node,PushRosNamespace
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import Command,LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    port_name_arg = LaunchConfiguration('port_name',default='/dev/agvpro_controller')
    namespace = LaunchConfiguration('namespace', default='')

    urdf_file = os.path.join(
        get_package_share_directory('agv_pro_description'),
        'urdf',
        'agv_pro.urdf'
    )

    robot_description_content = Command([
        'xacro ',
        urdf_file,
        ' namespace:=',
        PythonExpression(['"', namespace, '" + "/" if "', namespace, '" != "" else ""']),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'port_name', 
            default_value=port_name_arg,
            description='port name, e.g. ttyACM0'),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for nodes'),

        PushRosNamespace(namespace),

        Node(
            package='agv_pro_base',
            executable='agv_pro_node',
            name='agv_pro_node',
            output='screen',
            parameters=[{
                'port_name': port_name_arg,
                'namespace': namespace,             
            }],
            remappings=[('cmd_vel', '/cmd_vel')]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('lslidar_driver'),'launch'),
                '/lsn10p_launch.py'])
        )
    ])
