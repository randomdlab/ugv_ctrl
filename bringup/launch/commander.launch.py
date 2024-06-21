from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
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
            ("/in/odom", "/diff_drive/odometry"),
            ("/out/cmd_vel", "/diffbot_base_controller/cmd_vel"),
        ],
    )

    nodes = [
        node_commander,
    ]

    return LaunchDescription(declared_arguments + nodes)
