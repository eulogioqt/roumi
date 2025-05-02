from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2web',
            executable='server',
            name='r2w_server',
            parameters=[{
                'topics': "[['/camera/color/recognition', 'IMAGE']]"
            }],
            output='screen',
            prefix="xterm -hold -e",
            emulate_tty=True,
        ),
        Node(
            package='roumi_web',
            executable='api_rest',
            name='api_rest',
            output='screen'
        ),
        Node(
            package='roumi_web',
            executable='session_manager',
            name='session_manager',
            output='screen'
        ),
    ])