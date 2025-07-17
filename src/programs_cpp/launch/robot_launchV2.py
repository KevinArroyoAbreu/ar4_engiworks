from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

#SAME AS ROBOT LAUNCH, BUT WITH V2 POSITION SERVER


def generate_launch_description():
    # This is the driver of the robot
    # driver_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('annin_ar4_driver'),
    #             'launch',
    #             'driver.launch.py'
    #         )
    #     ),
    #     launch_arguments={'calibrate': 'True'}.items()
    # )

    # This is the rviz moveit configuration
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('annin_ar4_moveit_config'),
                'launch',
                'moveit.launch.py'
            )
        )
    )

    # Activate the camera node (change video channel if needed)
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{'video_device': '/dev/video4'}],
        remappings=[('image_raw', '/webcam/image_raw')]
    )

    # Launch the position tracker server
    position_server_v2 = Node(
        package='opencv_pkg',
        executable='position_server_v2',
        name='position_server_v2',
        namespace=''
    )

    

    # Create the launch description and populate
    return LaunchDescription([
        #driver_launch,
        moveit_launch,
        camera_node,
        position_server_v2
    ])