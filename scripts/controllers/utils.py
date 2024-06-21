from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry


def odom_to_yaw(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    r = R.from_quat([q.x, q.y, q.z, q.w])
    return r.as_euler("zyx")[0]
