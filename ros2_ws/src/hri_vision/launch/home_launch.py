from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hri_vision',
            executable='video',
            name='video',
            output='screen'
        ),
        Node(
            package='hri_vision',
            executable='detector',
            name='detector',
            output='screen'
        ),
        Node(
            package='hri_vision',
            executable='recognizer',
            name='recognizer',
            output='screen'
        ),
        Node(
            package='hri_vision',
            executable='logic',
            name='logic',
            output='screen',
        ),
        Node(
            package='ros2web',
            executable='server',
            name='r2w_server',
            parameters=[{
                'topics': "[['/camera/color/recognition', 'IMAGE'], ['/logic/info/actual_people', 'ACTUAL_PEOPLE']]"
            }],
            output='screen'
        ),
        Node(
            package='sancho_web',
            executable='sancho_web',
            name='sancho_web',
            output='screen'
        ),
        Node(
            package='sancho_web',
            executable='api_rest',
            name='api_rest',
            output='screen'
        ),
        Node(
            package='llm_tools',
            executable='llm',
            name='llm',
            output='screen'
        ),
        Node(
            package='sancho_ai',
            executable='sancho_ai',
            name='sancho_ai',
            output='screen'
        ),
    ])