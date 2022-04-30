#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import time
import tf2_ros
import tf2_geometry_msgs
from datetime import datetime

rospy.init_node("mocap_node")
INTERVAL = 0.1
device = rospy.get_param("~device")
master = mavutil.mavlink_connection(device)
buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)

receiving = True
last_received = None
last_sent = 0.0


def callback(msg: PoseStamped):
    global last_received, last_sent

    if last_received is None:
        rospy.loginfo("First pose received")
    last_received = time.time()

    if not receiving:
        rospy.loginfo("Pose reception resumed")
        receiving = True

    if time.time() - last_sent < INTERVAL:
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


def reception_check(_):
    global receiving
    if time.time() - last_received > 2 * INTERVAL:
        receiving = False
        rospy.logerror("Pose reception stopped")
        rospy.logwarn("Setting flight mode to land")
        master.mav.set_mav_cmd_do_set_mode(
            master.target_system,
            0,
            176,
            0,
            1,
        )


rospy.Subscriber(rospy.get_param("~topic"), PoseStamped, callback)
rospy.Timer(rospy.Duration(1), reception_check)
rospy.spin()
