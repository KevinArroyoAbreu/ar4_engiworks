from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='opencv_pkg',
            executable='load_image_from_camera',
            name='load_image_node',
            output='screen',
        )
    ])
