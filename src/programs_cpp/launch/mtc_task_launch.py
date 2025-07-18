from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Path to kinematics.yaml
    kinematics_path = os.path.join(
        get_package_share_directory("annin_ar4_moveit_config"),
        "config", "kinematics.yaml"
    )
    with open(kinematics_path, 'r') as f:
        kinematics_yaml_string = f.read()


    # Define your C++ node
    task_launch = Node(
        package='programs_cpp',
        executable='cubeRoutineV3',
        name='cubeRoutineV3',
        output='screen',
        namespace='',
        parameters=[
            {"robot_description_kinematics": kinematics_yaml_string}
        ]
    )

    return LaunchDescription([
        task_launch
    ])
