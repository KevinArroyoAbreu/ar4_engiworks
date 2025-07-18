import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    include_gripper = LaunchConfiguration('include_gripper', default='true')
    ar_model_config = LaunchConfiguration('ar_model', default='mk3')
    tf_prefix = LaunchConfiguration('tf_prefix', default='')

    # Robot description (xacro)
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([FindPackageShare('annin_ar4_description'), 'urdf', 'ar.urdf.xacro']),
        ' ',
        'ar_model:=', ar_model_config,
        ' ',
        'tf_prefix:=', tf_prefix,
        ' ',
        'include_gripper:=', include_gripper,
    ])
    robot_description = {'robot_description': robot_description_content}

    # Semantic description (xacro)
    robot_description_semantic_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([FindPackageShare('annin_ar4_moveit_config'), 'srdf', 'ar.srdf.xacro']),
        ' ',
        'name:=', ar_model_config,
        ' ',
        'tf_prefix:=', tf_prefix,
        ' ',
        'include_gripper:=', include_gripper,
    ])
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

    # Parameter files (no substitution macros inside these YAMLs)
    kinematics_param = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('annin_ar4_moveit_config'),
            'config',
            'kinematics.yaml'
        ]),
        allow_substs=True
    )
    joint_limits_param = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('annin_ar4_moveit_config'),
            'config',
            'joint_limits_fixed.yaml'
        ]),
        allow_substs=True
    )
    ompl_param = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('annin_ar4_moveit_config'),
            'config',
            'ompl_planning.yaml'
        ]),
        allow_substs=True
    )

    # Move Group node with all parameters
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_param,
            joint_limits_param,
            ompl_param,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Your custom MTC node (if it needs the same params)
    mtc_node = Node(
        package='programs_cpp',
        executable='cubeRoutineV3',
        name='cubeRoutineV3',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_param,
            joint_limits_param,
            ompl_param,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('include_gripper', default_value='true', description='Include gripper'),
        DeclareLaunchArgument('ar_model', default_value='mk3', description='Robot model'),
        DeclareLaunchArgument('tf_prefix', default_value='', description='TF prefix')
    ]

    return LaunchDescription(
        declared_arguments + [
            move_group_node,
            mtc_node,
        ]
    )
