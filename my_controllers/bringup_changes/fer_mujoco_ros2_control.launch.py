#!/usr/bin/env python3
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Launch Args
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    use_rviz = LaunchConfiguration("use_rviz")
    hand_control_type = LaunchConfiguration("hand_control_type")
    arm_control_type = LaunchConfiguration("arm_control_type")
    hand = LaunchConfiguration("hand")

    # Package Shares
    franka_mujoco_sim_bringup_pkg_share = FindPackageShare("franka_mujoco_sim_bringup")
    
    # Controllers
    controllers_yaml = PathJoinSubstitution([
        franka_mujoco_sim_bringup_pkg_share, "config", "franka_mujoco_controllers.yaml"
    ])
    
    # FER description 
    fer_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([franka_mujoco_sim_bringup_pkg_share, "urdf", "fer_mujoco.urdf.xacro"]),
            " ",
            "mujoco_control_type:=", arm_control_type,
            " ",
            "hand_control_type:=", hand_control_type,
            " ",
            "hand:=", hand,
            " ",
            "arm_prefix:=", "",
            " ",
            "xyz:='0 0 0'",
            " ",
            "rpy:='0 0 0'",
        ]
    )
    robot_description_str = fer_description_content.perform(context)
    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}

    # Robot State Publisher
    robot_state_pub = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # URDF to MJCF conversion
    world_model = PathJoinSubstitution([franka_mujoco_sim_bringup_pkg_share, "scenes", "base_world.xml"])
    urdf_to_mjcf_conversion = Node(
            package="mujoco_ros2_control",
            executable="robot_description_to_mjcf.sh",
            output="both",
            emulate_tty=True,
            arguments=[
                "--robot_description", robot_description_str,
                "--scene", world_model,
                "--publish_topic", "/mujoco_robot_description",
            ],
        )

    # MuJoCo ROS2 Control 
    mjc_ros2_control = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        emulate_tty=True,
        output="both",
        parameters=[
            {"use_sim_time": use_sim_time},
            controllers_yaml,
            ],
            on_exit=Shutdown(),
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("motion_control_handle/target_frame", "target_frame"),
            ("cartesian_compliance_controller/target_frame", "target_frame")
            ]
    )

    
    # Controllers 
    # Taken from https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/blob/ros2/cartesian_controller_simulation/launch/simulation.launch.py (last access: 23.03.2026)
    def controller_spawner(name, *args):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="both",
            arguments=[name] + [a for a in args] + 
                        ["--param-file", controllers_yaml, 
                        "--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time":use_sim_time}]
    )
    # Active controllers
    active_list = [
        "joint_state_broadcaster",
        "effort_forward_controller",
        "gripper_effort_controller",
    ]
    active_spawners = [controller_spawner(controller) for controller in active_list]

    # Inactive controllers
    # Note: joint_effort_traj_controller is NOT spawned here to avoid claiming
    # the effort interface while effort_forward_controller is active.
    # To switch to MoveIt mode, run:
    #   ros2 run controller_manager spawner joint_effort_traj_controller --param-file <controllers.yaml>
    #   ros2 control switch_controllers --deactivate effort_forward_controller --activate joint_effort_traj_controller --strict
    # JointTrajectoryControllers (joint_pos_traj_controller,
    # joint_effort_traj_controller) are NOT spawned here: they error at startup
    # because joint positions aren't available yet when they try to hold state.
    # Spawn them manually when needed:
    #   ros2 run controller_manager spawner <name> --param-file <yaml>
    inactive_list = [
        "gripper_position_controller",
        "cartesian_compliance_controller",
        "motion_control_handle"
    ]

    state = "--inactive"
    inactive_spawners = [
        controller_spawner(controller, state) for controller in inactive_list
    ]

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="mjc_rviz2",
        condition=IfCondition(use_rviz),
        output="both",
        arguments=["-d", PathJoinSubstitution(
            [franka_mujoco_sim_bringup_pkg_share, "etc", "default_rviz_conf.rviz"])],
        parameters=[{"use_sim_time":use_sim_time}]
    )
   
    return [robot_state_pub, urdf_to_mjcf_conversion, mjc_ros2_control, rviz] + active_spawners + inactive_spawners


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(generate_declared_arguments() + [OpaqueFunction(function=launch_setup)])


def generate_declared_arguments() -> List[DeclareLaunchArgument]:

    return [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If true, use simulated clock"
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="Level of logging for the ros2_nodes. Possible args ('debug', 'info', 'warn', 'error', 'fatal')."
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Use rviz2 or not. Defaults to true."
        ),
        DeclareLaunchArgument(
            "arm_control_type",
            default_value="effort",
            description="What interface to spawn the fer arm in the simulation with. Supported are: 'position', 'effort'"
        ),
        DeclareLaunchArgument(
            "hand",
            default_value="true",
            description="Whether to spawn the hand or not."
        ),
        DeclareLaunchArgument(
            "hand_control_type",
            default_value="effort",
            description="What interface to spawn the fer gripper in the simulation with. Supported are: 'position', 'effort'"
        )
    ]

