import numpy as np
from std_msgs.msg import Int16MultiArray, MultiArrayDimension


def convert_np_to_rosmsg(np_array):
    ros_msg = Int16MultiArray()

    array = []
    for i in range(np_array.shape[0]):
        for j in range(np_array.shape[1]):
            array.append(np_array[i, j])

    ros_msg.data = array
    return ros_msg


def convert_rosmsg_to_np(ros_msg):
    array = ros_msg.data
    img = np.zeros((240, 320))

    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            img[i, j] = array[img.shape[0] * i + j]

    return img
