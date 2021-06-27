#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node('base_to_center')
listener = tf.TransformListener()
pub = rospy.Publisher("/amcl_pose_center", PoseWithCovarianceStamped)

def callback(msg):
    p = PoseStamped()
    p.header.frame_id = "center_link"
    p.pose.orientation.w = 1.0
    listener.waitForTransform("/map", "/center_link", rospy.Time(0), rospy.Duration(3))
    # Obtain the center pose with respect to /map
    p = listener.transformPose("/map", p)

    # Change the original message pose so the stamp remains the same
    msg.pose.pose = p.pose
    pub.publish(msg)

    # print("base  : x = %2.3f, y = %2.3f"%(msg.pose.pose.position.x, msg.pose.pose.position.y))
    # print("center: x = %2.3f, y = %2.3f"%(p.pose.position.x, p.pose.position.y))

amcl_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback)
rospy.spin()
