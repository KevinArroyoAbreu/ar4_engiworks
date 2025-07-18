from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, FindExecutable
from launch_ros.substitutions import FindPackageShare

import yaml
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    abs_path = os.path.join(package_path, file_path)
    with open(abs_path, "r") as f:
        return yaml.safe_load(f)

def generate_launch_description():
    # Declare arguments for flexibility
    ar_model_arg = DeclareLaunchArgument('ar_model', default_value='mk3')
    tf_prefix_arg = DeclareLaunchArgument('tf_prefix', default_value='')
    include_gripper_arg = DeclareLaunchArgument('include_gripper', default_value='True')

    ar_model = LaunchConfiguration('ar_model')
    tf_prefix = LaunchConfiguration('tf_prefix')
    include_gripper = LaunchConfiguration('include_gripper')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('annin_ar4_description'), 'urdf', 'ar.urdf.xacro']),
        ' ar_model:=', ar_model,
        ' tf_prefix:=', tf_prefix,
        ' include_gripper:=', include_gripper,
    ])

    robot_description_param = {'robot_description': robot_description_content}

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('annin_ar4_moveit_config'), 'launch', 'moveit.launch.py')
        )
    )

    ompl_planning_yaml = load_yaml("annin_ar4_moveit_config", "config/ompl_planning.yaml")
    planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl", "pilz"],
        "ompl": ompl_planning_yaml,
    }

    robot_description_kinematics = load_yaml("annin_ar4_moveit_config", "config/kinematics.yaml")

    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('annin_ar4_moveit_config'), 'srdf', 'ar.srdf.xacro']),
        ' name:=', ar_model,
        ' tf_prefix:=', tf_prefix,
        ' include_gripper:=', include_gripper,
    ])
    robot_description_semantic_param = {'robot_description_semantic': robot_description_semantic_content}

    mtc_task_node = Node(
        package='programs_cpp',
        executable='cubeRoutineV3',
        output='screen',
        parameters=[
            robot_description_param,
            robot_description_semantic_param,
            planning_pipeline_config,
            {"robot_description_kinematics": robot_description_kinematics}
        ]
    )

    return LaunchDescription([
        ar_model_arg,
        tf_prefix_arg,
        include_gripper_arg,
        moveit_rviz_launch,
        mtc_task_node,
    ])
