import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Setup MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("annin_ar4")
        .robot_description("urdf/fake_ar.urdf.xacro")
        .semantic_robot_description("srdf/annin_ar4.srdf")
        .trajectory_execution("config/controllers.yaml")
        .planning_pipelines(["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )


    # Capabilities for task execution
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # RViz configuration
    rviz_config_file = os.path.join(
        get_package_share_directory("annin_ar4_moveit_config"),
        "rviz",
        "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Static TF publisher
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],  # Adjust frames!
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                get_package_share_directory("annin_ar4_moveit_config"),
                "config",
                "controllers.yaml",
            ),
        ],
        output="both",
    )

    # Move group node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # Controller spawners
    controller_names = [
         "joint_state_broadcaster",
        "joint_trajectory_controller",
        "gripper_controller",
    ]

    load_controllers = [
    ExecuteProcess(
        cmd=["ros2 run controller_manager spawner {} --controller-manager /controller_manager".format(controller)],
        shell=True,
        output="screen",
    )
    for controller in controller_names
]

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            ros2_control_node,
            run_move_group_node,
        ] + load_controllers
    )
