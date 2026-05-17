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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def launch_setup(context, *args, **kwargs):
    # Input parameters declaration
    arm = LaunchConfiguration("arm").perform(context)

    if arm == "ur":
        launch_file = PathJoinSubstitution([
            FindPackageShare("whi_moveit_cpp_bridge"),
            "launch",
            "ur_bringup_launch.py"
        ])
    elif arm == "jaka":
        launch_file = PathJoinSubstitution([
            FindPackageShare("whi_moveit_cpp_bridge"),
            "launch",
            "jaka_bringup_launch.py"
        ])
    else:
        raise RuntimeError(f"Unsupported arm: {arm}")

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "arm": LaunchConfiguration("arm"),
            "arm_model": LaunchConfiguration("arm_model"),
        }.items()
    )

    return [bringup_cmd]


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
