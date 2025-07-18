from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='opencv_pkg',
            executable='color_filtering',
            name='color_filtering',
            output='screen',
        )
    ])
