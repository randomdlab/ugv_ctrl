# Copyright 2021 Open Source Robotics Foundation, Inc.
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.substitutions import FindPackageShare


def prepend_prefix(s: str) -> PythonExpression:
    return PythonExpression(['"', LaunchConfiguration("prefix"), '"+"', s, '"'])


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix to add to the model name.",
        )
    )

    prefix = LaunchConfiguration("prefix")
    model_name = "mini_4wd"
    model_path = PathJoinSubstitution(
        [FindPackageShare("ugv_ctrl"), "models", model_name]
    )

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            prepend_prefix(model_name),
            "-allow_renaming",
            "true",
            "-z",
            "3.0",
        ],
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ugv_ctrl"),
                    f"models/{model_name}/urdf/all.xacro",
                ]
            ),
            " ",
            "use_gazebo:=True",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Get gz_bridge config file
    bridge_file_path = Command(
        [
            PathJoinSubstitution(
                [FindPackageShare("ugv_ctrl"), f"models/{model_name}/gz_bridge.py"]
            ),
            " ",
            prepend_prefix(model_name),
        ]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    node_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": bridge_file_path,
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_controller_mini_4wd",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    node_commander = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ugv_ctrl"), "/launch/commander.launch.py"]
        ),
    )

    nodes = [
        SetUseSimTime(True),
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        node_gz_bridge,
        robot_controller_spawner,
        node_commander,
    ]

    return LaunchDescription(nodes)
