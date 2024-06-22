#! /usr/bin/python3

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

import sys
import tempfile

model_name = sys.argv[1]
file_content = template.replace("{model_name}", model_name)

with tempfile.NamedTemporaryFile(delete=False) as f:
    f.write(file_content.encode())
    print(f.name)
