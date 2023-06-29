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


import numpy as np
from std_msgs.msg import Int16MultiArray


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
