#!/usr/bin/env python
import geometry_msgs
import rospy
import tf2_ros
from handrail_segmentation.msg import EkfState


def astrobee_pose_callback(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "body"
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z
    t.transform.rotation.x = msg.pose.orientation.x
    t.transform.rotation.y = msg.pose.orientation.y
    t.transform.rotation.z = msg.pose.orientation.z
    t.transform.rotation.w = msg.pose.orientation.w

    br.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node("astrobee_tf_broadcaster_node")
    rospy.Subscriber("/gnc/ekf", EkfState, astrobee_pose_callback, buff_size=2**24)
    rospy.spin()
