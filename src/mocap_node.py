#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import time
import tf2_ros
import tf2_geometry_msgs

rospy.init_node("mocap_feedback")
interval = int(rospy.get_param("~interval"))
device = rospy.get_param("~device")
master = mavutil.mavlink_connection(device)

# wait for connection
msg = None
while msg is not None:
    master.mav.ping_send(int(time.time() * 1e6), 0, 0, 0)
    rospy.loginfo(f"Establishing connection to {device} ...")
    msg = master.recv_match()
    time.sleep(0.5)
rospy.loginfo(f"Connection established to {device}")


# set origin and home position
master.mav.set_gps_global_origin_send(master.target_system, 599168569, 107284696, 0)
master.mav.set_home_position_send(
    master.target_system, 0, 0, 0, 0, 0, 0, [0, 0, 0, 0], 0, 0, 0
)

msg = PoseStamped()
buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)


def callback(untransformed_msg: PoseStamped):
    global msg
    transform = buffer.lookup_transform(
        # target frame:
        "local_ned",
        # source frame:
        untransformed_msg.header.frame_id,
        # get the tf at the time the pose was valid
        untransformed_msg.header.stamp,
        # wait for at most 1 second for transform, otherwise throw
        rospy.Duration(1.0),
    )
    msg = tf2_geometry_msgs.do_transform_pose(untransformed_msg, transform)


def transmit(_):
    time_usec = int(time.perf_counter_ns() * 1e3)

    o = msg.pose.orientation
    q = [o.w, o.x, o.y, o.z]

    p = msg.pose.position
    x, y, z = p.x, p.y, p.z

    master.mav.att_pos_mocap_send(time_usec, q, x, y, z)


rospy.Subscriber(rospy.get_param("~topic"), PoseStamped, callback)
rospy.Timer(rospy.Duration(0.2), transmit)
rospy.spin()
