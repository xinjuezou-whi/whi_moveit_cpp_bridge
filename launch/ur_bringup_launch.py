# Copyright 2026 WheelHub Intelligent
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from nav2_common.launch import RewrittenYaml
from pathlib import Path

def launch_setup(context, *args, **kwargs):
    # Input parameters declaration
    namespace = LaunchConfiguration('namespace')
    arm = LaunchConfiguration('arm').perform(context)
    arm_model = LaunchConfiguration('arm_model').perform(context)

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name=arm,
            package_name=f"{arm}_moveit_config"
        )
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": f"{arm}{arm_model}"})
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("whi_moveit_cpp_bridge"),
                "config",
                "moveit_cpp.yaml",
            )
        )
        .to_moveit_configs()
    )

    # Path to the config file
    config_file = PathJoinSubstitution([
        FindPackageShare('whi_moveit_cpp_bridge'),
        'config',
        'config.yaml'
    ])
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=config_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    config_arm_file = PathJoinSubstitution([
        FindPackageShare('whi_moveit_cpp_bridge'),
        'config',
        PythonExpression([
            "'config_' + '",
            arm,
            "' + '_' + '",
            arm_model,
            "' + '.yaml'"
        ])
    ])
    configured_arm_params = ParameterFile(
        RewrittenYaml(
            source_file=config_arm_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="whi_moveit_cpp_bridge",
        package="whi_moveit_cpp_bridge",
        executable="whi_moveit_cpp_bridge_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            configured_params,
            configured_arm_params,
        ],
    )

    # RViz
    # rviz_config_file = os.path.join(
    #     get_package_share_directory("moveit2_tutorials"),
    #     "launch",
    #     "moveit_cpp_tutorial.rviz",
    # )
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #     ],
    # )

    # Static TF
    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
    # )

    # Publish TF
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="both",
    #     parameters=[moveit_config.robot_description],
    # )

    # ros2_control using FakeSystem as hardware
    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("moveit_resources_panda_moveit_config"),
    #     "config",
    #     "ros2_controllers.yaml",
    # )
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[ros2_controllers_path],
    #     remappings=[
    #         ("/controller_manager/robot_description", "/robot_description"),
    #     ],
    #     output="both",
    # )

    launch_nodes = [
        # static_tf,
        # robot_state_publisher,
        # rviz_node,
        moveit_cpp_node,
    ]

    return launch_nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='',
            description='top-level namespace'),
        DeclareLaunchArgument('arm', default_value='ur',
            description='Arm brand'),
        DeclareLaunchArgument('arm_model', default_value='5e',
            description='Arm model'),
        OpaqueFunction(function=launch_setup)
    ])
