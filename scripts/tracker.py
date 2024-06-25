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


def update_tgts_stamp(
    targets: list[Odometry], stamp: rclpy.time.Time
) -> list[Odometry]:
    time_offset: int = (
        stamp.nanoseconds
        - rclpy.time.Time.from_msg(targets[0].header.stamp).nanoseconds
    )
    for tgt in targets:
        tgt_stamp = rclpy.time.Time.from_msg(tgt.header.stamp).nanoseconds
        tgt_stamp += time_offset
        tgt.header.stamp = rclpy.time.Time(nanoseconds=tgt_stamp).to_msg()
    return targets


class tracker_node(rclpy.node.Node):
    __odom_tgt_states: list[Odometry]
    __tgt_idx: int
    __msg_tgt_path: Path
    __msg_real_path: Path
    __cur_pose: Odometry
    __pub_tgt_path: rclpy.publisher.Publisher
    __pub_tgt_pose: rclpy.publisher.Publisher
    __pub_cmd: rclpy.publisher.Publisher
    __sub_odom: rclpy.subscription.Subscription
    __tmr_1hz: rclpy.timer.Timer
    __frame_id: str

    __is_first_odom_msg: bool

    def __init__(self):
        super().__init__("tracker_node")
        self.__msg_tgt_path = Path()
        self.__msg_real_path = Path()
        self.__tgt_idx = 0
        self.__load_param()

        self.__cur_pose = Odometry()
        self.__sub_odom = self.create_subscription(
            Odometry, "/in/odom", self.__cb_odom, 10
        )
        qos_pub_one_time = rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.__pub_tgt_path = self.create_publisher(
            Path, "/out/path/target", qos_pub_one_time
        )
        self.__pub_tgt_pose = self.create_publisher(PoseStamped, "/out/pose/target", 2)
        self.__pub_real_path = self.create_publisher(Path, "/out/path/real", 2)
        self.__pub_cmd = self.create_publisher(TwistStamped, "/out/cmd_vel", 2)
        # self.__tmr_1hz = self.create_timer(1, self.__cb_1hz)
        self.__tmr_ctrl = self.create_timer(0.03, self.__cb_ctrl)
        self.__tmr_update_tgt = self.create_timer(0.1, self.__cb_20hz_update_tgt)

        self.__is_first_odom_msg = False

        self.__pub_tgt_path.publish(self.__msg_tgt_path)

    def __load_param(self):
        self.declare_parameter("file_path", "")
        self.declare_parameter("topic", "")
        traj_file_path = str(self.get_parameter("file_path").value)
        traj_topic = str(self.get_parameter("topic").value)
        self.__odom_tgt_states, self.__msg_tgt_path = load_traj(
            traj_file_path, traj_topic
        )
        self.__frame_id = self.__msg_tgt_path.header.frame_id
        self.__msg_real_path.header.frame_id = self.__frame_id
        self.get_logger().info(
            f"Loaded {len(self.__odom_tgt_states)} poses from {traj_file_path} on topic {traj_topic}"
        )

    def __cb_odom(self, msg: Odometry):
        self.__cur_pose = msg
        p = PoseStamped()
        p.header = msg.header
        p.header.frame_id = "odom"
        p.pose.position = msg.pose.pose.position
        p.pose.orientation = msg.pose.pose.orientation
        self.__msg_real_path.poses.append(p)
        self.__pub_real_path.publish(self.__msg_real_path)
        if not self.__is_first_odom_msg:
            self.__is_first_odom_msg = True
            update_tgts_stamp(
                self.__odom_tgt_states, rclpy.time.Time.from_msg(msg.header.stamp)
            )

    def __cb_1hz(self):
        self.__msg_tgt_path.header.stamp = self.get_clock().now().to_msg()
        self.__pub_tgt_path.publish(self.__msg_tgt_path)

    def __calc_cmd(self) -> tuple[float, float]:
        from controllers.easy import calc_cmd

        length = min(5, len(self.__odom_tgt_states) - self.__tgt_idx)
        start = self.__tgt_idx
        end = start + length
        vl, va = calc_cmd(self.__odom_tgt_states[start:end], self.__cur_pose)
        return vl, va

    def __update_target(self):
        def state_diff() -> tuple[float, float, float]:
            tgt = self.__odom_tgt_states[self.__tgt_idx]
            time_tgt = rclpy.time.Time.from_msg(tgt.header.stamp)
            time_cur = rclpy.time.Time.from_msg(self.__cur_pose.header.stamp)
            x_diff = tgt.pose.pose.position.x - self.__cur_pose.pose.pose.position.x
            y_diff = tgt.pose.pose.position.y - self.__cur_pose.pose.pose.position.y
            pose_diff = math.sqrt(x_diff**2 + y_diff**2)
            return time_tgt, time_cur, pose_diff

        prv_idx = self.__tgt_idx
        while self.__tgt_idx < len(self.__odom_tgt_states):
            t_tgt, t_cur, p_diff = state_diff()
            # if t_tgt > t_cur or p_diff < 0.2:
            #     tgt = self.__odom_tgt_states[self.__tgt_idx]
            #     msg = PoseStamped()
            #     msg.header = self.__cur_pose.header
            #     msg.pose = tgt.pose.pose
            #     self.__pub_tgt_pose.publish(msg)
            #     # self.get_logger().info(
            #     #     f"Target updated to {msg.position.x}, {msg.position.y}"
            #     # )
            #     break
            # self.__tgt_idx += 1
            if p_diff < 1.0:
                self.__tgt_idx += 1
                continue
            if self.__tgt_idx > prv_idx:
                tgt = self.__odom_tgt_states[self.__tgt_idx]
                msg = PoseStamped()
                msg.header = self.__cur_pose.header
                msg.pose = tgt.pose.pose
                self.__pub_tgt_pose.publish(msg)
                self.get_logger().info(
                    f"Target updated to {msg.pose.position.x}, {msg.pose.position.y}"
                )
            break
        if self.__tgt_idx >= len(self.__odom_tgt_states):
            self.get_logger().info("Target reached")
            exit(0)

    def __cb_20hz_update_tgt(self):
        self.__tgt_idx += 1
        if self.__tgt_idx >= len(self.__odom_tgt_states):
            self.__tgt_idx = len(self.__odom_tgt_states) - 1
            self.get_logger().info("Target reached")
            self.__tmr_update_tgt.cancel()
            return
        tgt = self.__odom_tgt_states[self.__tgt_idx]
        msg = PoseStamped()
        msg.header = self.__cur_pose.header
        msg.pose = tgt.pose.pose
        self.__pub_tgt_pose.publish(msg)
        # self.get_logger().info(
        #     f"Target updated to {msg.pose.position.x}, {msg.pose.position.y}"
        # )

    def __cb_ctrl(self):
        if not self.__is_first_odom_msg:
            return
        vl, va = self.__calc_cmd()
        cmd = TwistStamped()
        cmd.header.frame_id = self.__frame_id
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = vl
        cmd.twist.angular.z = va
        self.__pub_cmd.publish(cmd)
        # self.__update_target()


def main(args=None):
    rclpy.init(args=args)
    node = tracker_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
