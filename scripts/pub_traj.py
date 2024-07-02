#! /usr/bin/env python3

import rclpy
import rclpy.node
import rclpy.publisher
import rclpy.qos
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import rclpy.subscription
import rclpy.time
import rclpy.timer
import copy


def load_traj(traj_file_path: str, topic: str) -> tuple[list[Odometry], Path]:
    import pathlib
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    odoms = []
    path = Path()
    typestore = get_typestore(Stores.LATEST)
    with AnyReader(
        [pathlib.Path(traj_file_path)], default_typestore=typestore
    ) as reader:
        connections = [x for x in reader.connections if x.topic == topic]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            odom = Odometry()
            odom.header.frame_id = msg.header.frame_id
            odom.header.stamp.sec = msg.header.stamp.sec
            odom.header.stamp.nanosec = msg.header.stamp.nanosec
            odom.pose.pose.position.x = msg.pose.pose.position.x
            odom.pose.pose.position.y = msg.pose.pose.position.y
            odom.pose.pose.position.z = msg.pose.pose.position.z
            odom.pose.pose.orientation.x = msg.pose.pose.orientation.x
            odom.pose.pose.orientation.y = msg.pose.pose.orientation.y
            odom.pose.pose.orientation.z = msg.pose.pose.orientation.z
            odom.pose.pose.orientation.w = msg.pose.pose.orientation.w
            odom.twist.twist.linear.x = msg.twist.twist.linear.x
            odom.twist.twist.linear.y = msg.twist.twist.linear.y
            odom.twist.twist.linear.z = msg.twist.twist.linear.z
            odom.twist.twist.angular.x = msg.twist.twist.angular.x
            odom.twist.twist.angular.y = msg.twist.twist.angular.y
            odom.twist.twist.angular.z = msg.twist.twist.angular.z
            odoms.append(odom)
            p = PoseStamped()
            p.header = copy.deepcopy(odom.header)
            p.pose.position = copy.deepcopy(odom.pose.pose.position)
            p.pose.orientation = copy.deepcopy(odom.pose.pose.orientation)
            path.poses.append(p)
    path.header = path.poses[0].header
    return odoms, path


class pub_traj_node(rclpy.node.Node):
    def __init__(self):
        super().__init__("pub_traj_node")
        self.declare_parameter("file_path", "")
        self.declare_parameter("topic", "")
        traj_file_path = str(self.get_parameter("file_path").value)
        traj_topic = str(self.get_parameter("topic").value)
        self.__odom_tgt_states, self.__msg_tgt_path = load_traj(
            traj_file_path, traj_topic
        )
        self.__tmr_pub_traj = self.create_timer(0.8, self.__cb_pub_traj)
        self.__pub_odom = self.create_publisher(Odometry, "target", 5)
        self.__pub_pose = self.create_publisher(PoseStamped, "target_pose", 5)
        qos_pub_one_time = rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.__pub_path = self.create_publisher(Path, "target_path", qos_pub_one_time)
        self.__pub_path.publish(self.__msg_tgt_path)

    def __cb_pub_traj(self):
        if len(self.__odom_tgt_states) <= 0:
            self.get_logger().info("traj replay finish, exit")
            exit(0)
        tgt = self.__odom_tgt_states[0]
        tgt.header.stamp = self.get_clock().now().to_msg()
        self.__odom_tgt_states.pop(0)
        msg = PoseStamped()
        msg.header = tgt.header
        msg.pose = tgt.pose.pose
        self.__pub_pose.publish(msg)
        self.__pub_odom.publish(tgt)


def main(args=None):
    rclpy.init(args=args)
    node = pub_traj_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
