#! /usr/bin/env python3

import rclpy
import rclpy.node
import rclpy.publisher
import rclpy.qos
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
import rclpy.subscription
import rclpy.time
import rclpy.timer
import math
import copy
from controllers.mpc import calc_cmd


class tracker_node(rclpy.node.Node):
    __msg_real_path: Path
    __cur_pose: Odometry
    __pub_cmd: rclpy.publisher.Publisher
    __sub_odom: rclpy.subscription.Subscription
    __sub_tgt: rclpy.subscription.Subscription
    __frame_id: str

    def __init__(self):
        super().__init__("tracker_node")
        self.__tgt = Odometry()
        self.__msg_real_path = Path()
        self.__cur_pose = Odometry()
        self.__sub_odom = self.create_subscription(
            Odometry, "/in/odom", self.__cb_odom, 10
        )
        self.__sub_tgt = self.create_subscription(Odometry, "target", self.__cb_tgt, 1)
        self.__pub_real_path = self.create_publisher(Path, "/out/path/real", 2)
        self.__pub_cmd = self.create_publisher(TwistStamped, "/out/cmd_vel", 2)
        self.__pub_tgt_pose = self.create_publisher(PoseStamped, "/out/pose/track", 2)
        self.__tmr_ctrl = self.create_timer(0.03, self.__cb_ctrl)
        self.__is_get_tgt = False
        self.__msg_real_path.header.frame_id = "odom"

    def __cb_tgt(self, msg: Odometry):
        self.__tgt = msg
        self.__is_get_tgt = True

    def __cb_odom(self, msg: Odometry):
        self.__cur_pose = msg
        p = PoseStamped()
        p.header = msg.header
        p.header.frame_id = "odom"
        p.pose.position = msg.pose.pose.position
        p.pose.orientation = msg.pose.pose.orientation
        self.__msg_real_path.poses.append(p)
        self.__pub_real_path.publish(self.__msg_real_path)

    def __cb_ctrl(self):
        if not self.__is_get_tgt:
            return
        vl, va = calc_cmd([self.__tgt], self.__cur_pose)
        cmd = TwistStamped()
        cmd.header.frame_id = self.__tgt.header.frame_id
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = vl
        cmd.twist.angular.z = va
        self.__pub_cmd.publish(cmd)
        msg = PoseStamped()
        msg.header = cmd.header
        msg.pose = self.__tgt.pose.pose
        self.__pub_tgt_pose.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = tracker_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
