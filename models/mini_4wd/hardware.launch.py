# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix to add to the model name.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "file_path",
            default_value="",
            description="Ros2 bag folder path which stores target path.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "topic",
            default_value="",
            description="Topic of target path.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    model_name = "mini_4wd"
    prefix_model_name_py_expr = PythonExpression(
        ['"', LaunchConfiguration("prefix"), '"+"', model_name, '"']
    )

    controller_cfg_file_path = PathJoinSubstitution(
        [
            FindPackageShare("ugv_ctrl"),
            f"models/{model_name}/control_cfg.yml",
        ]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_cfg_file_path, ("base_frame_id", "dummy")],
        output="both",
    )

    # robot state publisher
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
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("ugv_ctrl"),
            f"models/{model_name}/rviz.rviz",
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    controller_name = "diff_controller_mini_4wd"
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller_name,
            "--controller-manager",
            "/controller_manager",
        ],
    )

    node_commander = Node(
        package="ugv_ctrl",
        executable="tracker.py",
        name="tracker",
        output="screen",
        parameters=[
            {"file_path": LaunchConfiguration("file_path")},
            {"topic": LaunchConfiguration("topic")},
        ],
        remappings=[
            ("/in/odom", f"{controller_name}/odom"),
            ("/out/cmd_vel", f"{controller_name}/cmd_vel"),
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        node_commander,
    ]

    return LaunchDescription(declared_arguments + nodes)
