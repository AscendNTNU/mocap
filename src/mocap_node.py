#! /usr/bin/env python
from cmath import log
import rospy
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import time
import tf2_ros
import tf2_geometry_msgs
from datetime import datetime

rospy.init_node("mocap_node")
interval = float(rospy.get_param("~interval"))
device = rospy.get_param("~device")
master = mavutil.mavlink_connection(device)

buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)
last_sent = 0.0
last_received = 0.0
last_logerr = 0.0


def ping(timeout=2):
    """Pings master and returns wether a reponse was gotten within the timeout"""
    master.mav.ping_send(0, 0, 0, 0)
    sent = time.time()
    while time.time() - sent < timeout:
        if master.recv_match() is not None:
            return True
    return False


def set_home_and_origin():
    master.mav.set_gps_global_origin_send(master.target_system, 599168569, 107284696, 0)
    master.mav.set_home_position_send(
        master.target_system, 0, 0, 0, 0, 0, 0, [0, 0, 0, 0], 0, 0, 0
    )


def callback(msg: PoseStamped):
    global last_received, last_sent
    last_received = time.time()

    # Ensure that mocap recepetion failure have been reported before reporting that things work again
    if time.time() - last_received > 5 * interval + 1:
        rospy.logwarn("Pose received again")

    if time.time() - last_sent < interval:
        return

    transform = buffer.lookup_transform(
        # target frame:
        "local_ned",
        # source frame:
        msg.header.frame_id,
        # get the tf at the time the pose was valid
        msg.header.stamp,
        # wait for at most 1 second for transform, otherwise throw
        rospy.Duration(1.0),
    )

    msg = tf2_geometry_msgs.do_transform_pose(msg, transform)

    o = msg.pose.orientation
    q = [o.w, o.x, o.y, o.z]

    p = msg.pose.position
    x, y, z = p.x, p.y, p.z

    master.mav.att_pos_mocap_send(0, q, x, y, z)
    last_sent = time.time()


def check_mavlink_connection(_):
    if not ping():
        rospy.logerr(f"Lost connection to {device}")
        while not ping():
            pass
        rospy.logwarn(f"Connection reestablished to {device}")
        set_home_and_origin()
    else:
        rospy.loginfo(f"Connection to {device} is still good")


def check_mocap_reception(_):
    global last_logerr
    since_last_pose = time.time() - last_received
    if since_last_pose > 5 * interval and time.time() - last_logerr > 30:
        rospy.logerr(f"No pose received in the last {since_last_pose} seconds")
        last_logerr = time.time()


if not ping():
    rospy.logerr(f"Failed to connect to {device}")
    while not ping():
        pass
    rospy.logwarn(f"Connection established to {device}")
    set_home_and_origin()
else:
    rospy.loginfo(f"Connection established to {device}")


set_home_and_origin()

rospy.Subscriber(rospy.get_param("~topic"), PoseStamped, callback)
rospy.Timer(rospy.Duration(1), check_mavlink_connection)
rospy.Timer(rospy.Duration(1), check_mocap_reception)
rospy.spin()
