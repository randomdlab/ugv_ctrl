from nav_msgs.msg import Odometry
import math
from controllers.utils import odom_to_yaw
import numpy as np


def from_two_vec(v1: np.ndarray, v2: np.ndarray) -> float:
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    d = np.dot(v1, v2)
    rad = 0.0
    if d >= 1.0:
        rad = 0.0
    elif d <= -1.0:
        rad = math.pi
    else:
        rad = math.acos(d)
    return rad


def calc_cmd(targets: list[Odometry], pose: Odometry) -> tuple[float, float]:
    tgt = targets[0]
    x_diff = tgt.pose.pose.position.x - pose.pose.pose.position.x
    y_diff = tgt.pose.pose.position.y - pose.pose.pose.position.y
    pose_diff = math.sqrt(x_diff**2 + y_diff**2)
    theta_r = math.atan2(y_diff, x_diff)
    theta = odom_to_yaw(pose)
    theta_d = theta_r - theta
    while theta_d > math.pi:
        theta_d -= 2 * math.pi
    while theta_d < -math.pi:
        theta_d += 2 * math.pi
    vl = 0.0
    va = theta_d * 1.0
    # print(f"theta_r: {theta_r}, theta: {theta}, theta_d: {theta_d}")
    if abs(va) < 0.2:
        vl = min(4.0, pose_diff)
    elif abs(va) < 0.5:
        vl = min(3.0, pose_diff)
    else:
        vl = min(2.0, pose_diff)
    return vl, va
