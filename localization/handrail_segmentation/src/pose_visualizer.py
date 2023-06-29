#!/usr/bin/env python3
#
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.


# ROS imports
import geometry_msgs
import rospy
import tf2_ros
import tf.transformations as tt


def populate_transform_stamped(pose, frame):
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = frame
    quat = tt.quaternion_from_euler(pose[3], pose[4], pose[5])
    t.transform.translation.x = pose[0]
    t.transform.translation.y = pose[1]
    t.transform.translation.z = pose[2]
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    return t


def talker():
    rospy.init_node("posevisualizer")
    br = tf2_ros.TransformBroadcaster()
    t1 = populate_transform_stamped(
        [
            0.8962491872,
            -0.08222659941,
            -0.7585849084,
            -0.006926715158,
            0.0815173987,
            3.09210203,
        ],
        "n-c 1",
    )
    t2 = populate_transform_stamped(
        [
            0.9418942474,
            0.007622090758,
            -0.648843869,
            -0.1826375438,
            -0.01263754379,
            3.233312934,
        ],
        "n-c 2",
    )
    t3 = populate_transform_stamped(
        [
            0.855947264,
            0.01593218138,
            -0.6906582566,
            -0.09057921926,
            -0.09355557577,
            2.806911507,
        ],
        "n-c 3",
    )
    t4 = populate_transform_stamped(
        [
            0.8628785891,
            -0.03472339497,
            -0.8459514592,
            0.02349845393,
            0.03782591857,
            2.511929285,
        ],
        "c 1",
    )
    t5 = populate_transform_stamped(
        [
            0.971192742,
            0.05386297882,
            -0.9375246469,
            -0.01398405314,
            -0.007718710198,
            3.201151398,
        ],
        "c 2",
    )
    t6 = populate_transform_stamped(
        [
            0.937186718,
            0.03744370863,
            -0.8865978122,
            -0.113142966,
            0.1363806412,
            2.950612599,
        ],
        "c 3",
    )
    true = populate_transform_stamped([0.93, 0.03, -0.9, 0, 0, 3.141592], "true")

    while not rospy.is_shutdown():
        br.sendTransform(t1)
        br.sendTransform(t2)
        br.sendTransform(t3)
        br.sendTransform(t4)
        br.sendTransform(t5)
        br.sendTransform(t6)
        br.sendTransform(true)


if __name__ == "__main__":

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
