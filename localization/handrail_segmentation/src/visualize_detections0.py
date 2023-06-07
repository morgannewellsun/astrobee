#!/usr/bin/env python3
import sys

import rospy
from sensor_msgs.msg import Image as ROSImage

sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
import os

import cv2
import numpy as np
from rospkg import RosPack

package = RosPack()
package_path = package.get_path("object_pose_estimation")


class VisualizeImageDetections(object):
    def __init__(self):
        # Params
        self.image = None
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.image_sub = rospy.Subscriber(
            "det/features/annotated_img",
            ROSImage,
            self.imageCb,
            queue_size=1,
            buff_size=2**24,
        )

    def imageCb(self, img_msg):
        img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
            img_msg.height, img_msg.width, -1
        )[:, :, ::-1]
        cv2.imshow("Annotated Image", img)
        cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("visualize_detections", anonymous=True)
    my_node = VisualizeImageDetections()
    rospy.spin()
