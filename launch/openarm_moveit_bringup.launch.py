# Copyright 2026 WheelHub Intelligent
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from nav2_common.launch import RewrittenYaml
from pathlib import Path

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    use_rviz = LaunchConfiguration('use_rviz').perform(context)

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="openarm_bimanual",
            package_name="openarm_bimanual_moveit_config"
        )
        .robot_description(
            file_path=Path("config") / "openarm_v2.0" / "openarm_bimanual.urdf.xacro"
        )
        .robot_description_semantic(
            file_path=Path("config") / "openarm_v2.0" / "openarm_bimanual.srdf"
        )
        .robot_description_kinematics(
            file_path=Path("config") / "openarm_v2.0" / "kinematics.yaml"
        )
        .joint_limits(
            file_path=Path("config") / "openarm_v2.0" / "joint_limits.yaml"
        )
        .trajectory_execution(
            file_path=Path("config") / "openarm_v2.0" / "moveit_controllers.yaml"
        )
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("whi_moveit_cpp_bridge"),
                "config",
                "moveit_cpp.yaml",
            )
        )
        .to_moveit_configs()
    )

    config_file = os.path.join(
        get_package_share_directory("whi_moveit_cpp_bridge"), "config", "config.yaml"
    )
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=config_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    config_left_file = os.path.join(
        get_package_share_directory("whi_moveit_cpp_bridge"), "config", "config_openarm_left.yaml"
    )
    configured_left_params = ParameterFile(
        RewrittenYaml(
            source_file=config_left_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    config_right_file = os.path.join(
        get_package_share_directory("whi_moveit_cpp_bridge"), "config", "config_openarm_right.yaml"
    )
    configured_right_params = ParameterFile(
        RewrittenYaml(
            source_file=config_right_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    common_remap_pairs = [
        "tcp_pose",
        "joint_pose",
        "joint_names",
        "tcp_difference",
        "tcp_current",
        "joint_current",
        "abort_execution",
        "moveit_cpp_state",
    ]

    moveit_cpp_node_left = Node(
        name="whi_moveit_cpp_bridge_left",
        package="whi_moveit_cpp_bridge",
        executable="whi_moveit_cpp_bridge_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            configured_params,
            configured_left_params,
        ],
        remappings=[(name, f"left_arm/{name}") for name in common_remap_pairs],
    )

    moveit_cpp_node_right = Node(
        name="whi_moveit_cpp_bridge_right",
        package="whi_moveit_cpp_bridge",
        executable="whi_moveit_cpp_bridge_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            configured_params,
            configured_right_params,
        ],
        remappings=[(name, f"right_arm/{name}") for name in common_remap_pairs],
    )

    nodes = [moveit_cpp_node_left, moveit_cpp_node_right]

    if use_rviz == "true":
        # 标准 move_group：给 RViz MotionPlanning 插件做后端，
        # 拖拽的交互式 marker、Plan/Execute 都靠它
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
        )

        rviz_config_path = str(
            Path(get_package_share_directory("openarm_bimanual_moveit_config"))
            / "config" / "openarm_v2.0" / "moveit.rviz"
        )
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        )

        nodes += [move_group_node, rviz_node]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='',
            description='top-level namespace'),
        DeclareLaunchArgument('use_rviz', default_value='true',
            description='if strat rviz'),
        OpaqueFunction(function=launch_setup)
    ])