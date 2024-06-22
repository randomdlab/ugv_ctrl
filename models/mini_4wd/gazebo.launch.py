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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
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
import tempfile

template = """
---
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
- ros_topic_name: "/gz/{model_name}/odometry"
  gz_topic_name: "/model/{model_name}/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
- ros_topic_name: "/joint_states"
  gz_topic_name: "/world/empty/model/{model_name}/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS
- ros_topic_name: "/tf"
  gz_topic_name: "/model/{model_name}/pose"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS
- ros_topic_name: "/tf_static"
  gz_topic_name: "/model/{model_name}/pose_static"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS
"""


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

    model_name = "mini_4wd"
    prefix_model_name_py_expr = PythonExpression(
        ['"', LaunchConfiguration("prefix"), '"+"', model_name, '"']
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
            prefix_model_name_py_expr,
            "-z",
            "0.5",
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
            prefix_model_name_py_expr,
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
            (
                "/in/odom",
                PythonExpression(
                    ['"/gz/"', '+"', prefix_model_name_py_expr, '"+', '"/odometry"']
                ),
            ),
            ("/out/cmd_vel", f"{controller_name}/cmd_vel"),
        ],
    )

    delay_clean_tmp_file = RegisterEventHandler(
        OnProcessExit(
            target_action=node_gz_bridge,
            on_exit=[ExecuteProcess(cmd=["rm", bridge_file_path])],
        )
    )

    nodes = [
        SetUseSimTime(True),
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        node_gz_bridge,
        robot_controller_spawner,
        node_commander,
        delay_clean_tmp_file,
    ]

    return LaunchDescription(nodes)
