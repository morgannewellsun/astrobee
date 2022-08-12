#!/usr/bin/env python3

from __future__ import division


# ROS imports
import rospy
import std_msgs.msg
from rospkg import RosPack
from std_msgs.msg import UInt8, Int16MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image as ROSImage
from sensor_msgs.msg import PointCloud2, CameraInfo
import sensor_msgs
import sensor_msgs.point_cloud2 as pc2


# Python imports
import sys
import argparse
from PIL import Image
import numpy as np
from tqdm import tqdm
import os, cv2
from cv_bridge import CvBridge
from utils.undistorter import Undistorter
from utils.np_to_multiarray import convert_np_to_rosmsg

# Deep learning imports
import torch
import torchvision
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from utils.transformation import TransformAlignment
from sklearn.cluster import DBSCAN
import timeit

from geometry_msgs.msg import Pose
import utils.icp

import open3d as o3d
import numpy as np
from utils.transformation import TransformAlignment
import utils.icp as icp
from utils.converter import *
import copy
from probreg import cpd
from scipy.spatial.transform import Rotation as R

import timeit



from torchvision import transforms
convert_tensor = transforms.ToTensor()

# Get package path in file directory
package = RosPack()
package_path = package.get_path('handrail_segmentation')





def to_multiarray_i16(np_array):
    multiarray = Int16MultiArray()
    multiarray.layout.dim = [MultiArrayDimension('dim%d' % i,
                                                 np_array.shape[i],
                                                 np_array.shape[i] * np_array.dtype.itemsize) for i in range(np_array.ndim)];
    multiarray.data = np_array.reshape([1, -1])[0].tolist();
    return multiarray




def post_process(detections, num_detections, c_thresh = 0.75):
    p_detections = []
    for i in range(num_detections):
        if detections['scores'][i] > c_thresh:
            detection = {"bbox": detections['boxes'].detach().numpy()[i].reshape(4,),
                         "mask": detections['masks'].detach().numpy()[i].reshape(240, 320),
                         "label": detections['labels'].detach().numpy()[i]}
            p_detections.append(detection)
            rospy.loginfo("~~~~~~Handrail detected~~~~~~")

    return p_detections

def get_trained_model(weights_path, num_classes = 5):
    # load an instance segmentation model pre-trained on COCO
    model = torchvision.models.detection.maskrcnn_resnet50_fpn()
    # replace the pre-trained head with a new one
    # get number of input features for the classifier
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)

    # now get the number of input features for the mask classifier
    in_features_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels
    hidden_layer = 256
    # and replace the mask predictor with a new one
    model.roi_heads.mask_predictor = MaskRCNNPredictor(in_features_mask,
                                                       hidden_layer,
                                                       num_classes)


    model.load_state_dict(torch.load(weights_path))

    return model

def execute_global_registration(src, target, voxel_size=0.01):
    source_down = src.voxel_down_sample(voxel_size=voxel_size)
    target_down = target.voxel_down_sample(voxel_size=voxel_size)
    result, _, _= cpd.registration_cpd(source_down, target_down, maxiter = 50)

    return result

class HandrailDetectorManager():
    def __init__(self):
        self.simulation = True
        # Load weights parameter
        weights_name = rospy.get_param('~weights_name', 'finetuned_ckpt_199.pth')
        self.weights_path = os.path.join(package_path, 'src/weights', weights_name)
        rospy.loginfo("Found weights, loading %s", self.weights_path)

        # Raise error if it cannot find the model
        if not os.path.isfile(self.weights_path):
            raise IOError(('{:s} not found.').format(self.weights_path))

        # Load image parameter and confidence threshold
        self.image_topic = rospy.get_param('~image_topic', '/hw/cam_dock_color')
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/hw/depth_perch/points')

        self.nms_th = rospy.get_param('~nms_th', 0.9)

        # Load publisher topics
        self.segmentation_mask = rospy.get_param('~detected_objects_topic', "det/mask")
        self.published_annotated_img_topic = rospy.get_param('~detections_image_topic', "det/features/annotated_img")
        self.publish_image = rospy.get_param('~publish_image', True)

        # Initialize width and height
        self.h = 240
        self.w = 320
        self.undist_ = Undistorter(simulation=self.simulation)
        self.align_transformer = TransformAlignment()

        self.model = get_trained_model(self.weights_path)
        device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        self.model.to(torch.device('cpu'))

        self.model.eval() # Set in evaluation mode
        rospy.loginfo("Deep neural network loaded")

        self.coloring_scheme = {1: [55, 255, 20],
                       2: [0, 200, 172],
                       3: [0, 106, 200],
                       4: [0, 173, 2]}



        # Define subscribers
        self.imgMsg = None
        self.pcMsg = None
        self.busy = False

        self.registered_handrail_o3d = o3d.io.read_point_cloud('/home/anaveen/Documents/nasa_ws/astrobee-detection-pipeline/src/handrail_segmentation/src/reference_pointclouds/handrail_30.pcd')
        self.registered_handrail = np.asarray(self.registered_handrail_o3d.points)

        self.image_sub = rospy.Subscriber(self.image_topic, ROSImage, self.imageCb, queue_size = 1, buff_size = 2**24)
        self.pointcloud_sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback, queue_size = 1, buff_size = 2**24)

        # Define publishers
        self.pub_ = rospy.Publisher(self.segmentation_mask, ROSImage, queue_size=1)
        self.pub_viz_ = rospy.Publisher(self.published_annotated_img_topic, ROSImage, queue_size=10)
        self.bridge = CvBridge()
        rospy.loginfo("Launched node for handrail segmentation")


    def preprocessImage(self, imgIn):
        imgIn = cv2.cvtColor(imgIn, cv2.COLOR_BGR2RGB)
        imgIn = cv2.resize(imgIn, (320, 240), interpolation = cv2.INTER_AREA)
        imgIn = self.undist_.undistort(imgIn)
        return [convert_tensor(imgIn)], imgIn

    def add_colored_to_image(self, image, colored):
        return cv2.addWeighted(cv2.resize(image, (colored.shape[1], colored.shape[0]))
                                .astype(np.uint8), 1,
                                colored.astype(np.uint8), 0.5,
                                0, cv2.CV_32F)




    def convert_mask_to_image(self, mask, label):
        colored_map = np.array(Image.fromarray(mask).convert("RGB"))
        colored_map[np.where(mask == label)] = self.coloring_scheme[label]
        return colored_map


    def visualize(self, image, bbox, mask, label):
        colored_map = self.convert_mask_to_image(mask, label)
        colored_image = self.add_colored_to_image(image, colored_map)
        return cv2.rectangle(colored_image, (int(np.floor(bbox[0])), int(np.floor(bbox[1]))), (int(np.ceil(bbox[2])), int(np.ceil(bbox[3]))), self.coloring_scheme[label], 1)

    def process_mask(self, mask):
        colored_map = np.array(Image.fromarray(mask).convert("RGB"))
        return colored_map


    def imageCb(self, data):
        self.imgMsg = data
        self.process()

    def pointcloud_callback(self, msg):
        # "Store" the message received.
        self.pcMsg = msg

        # Compute stuff.
        self.process()


    def process(self):
        if self.imgMsg is not None and self.pcMsg is not None and not self.busy:
            self.busy = True
            rospy.loginfo("Recieved image from dock cam")
            # Convert the image to OpenCV
            start = timeit.default_timer()
            imgDist = np.asarray(self.bridge.imgmsg_to_cv2(self.imgMsg, 'bgr8'))
            imgIn, annotated_img = self.preprocessImage(imgDist)

            dims = (annotated_img.shape[0], annotated_img.shape[1])
            # torch.cuda.synchronize()
            detections = self.model(imgIn)[0]
            n = len(detections['labels'])
            detections = post_process(detections, n)

            mask_ = np.zeros(dims).astype(np.uint8)
            for detection in detections:

                bbox, mask, label = detection.values()

                np.place(mask, mask > self.nms_th, label)
                np.place(mask, mask <= self.nms_th, 0)

                mask_ = mask_ + mask

            stop = timeit.default_timer()
            rospy.loginfo("~~~~~~~~~~~~~Handrail Mask RCNN took is " + str(stop - start) + " seconds.~~~~~~~~~~~~~~~~")
            start = timeit.default_timer()
            points_ = self.align_transformer.perch_to_dock_transform(self.pcMsg)
            points_np = self.align_transformer.convert_pc_msg_to_np(points_)
            n_points = points_np.shape[0]

            K = self.undist_.K
            f = K[0][0]
            cx = K[0][2]
            cy = K[1][2]

            min_pixel_z = 100*np.ones(mask_.shape)
            pixel_point_map = -1*np.ones(mask_.shape)
            for i in range(n_points):
                pt = points_np[i].reshape(3, )
                px = int(np.floor(f*pt[0]/pt[2] + cx))
                py = int(np.round(f*pt[1]/pt[2] + cy))

                if py >= mask_.shape[0] or px >= mask_.shape[1]:
                    continue

                if py < 0 or px < 0:
                    continue

                # Different Approach
                if min_pixel_z[py][px] > pt[2]:
                    min_pixel_z[py][px] = pt[2]
                    pixel_point_map[py][px] = i

            handrail_pixels = np.where(mask_ > 0)
            points_handrail_indices = np.unique(pixel_point_map[handrail_pixels]).astype(int)
            if points_handrail_indices[0] == -1:
                points_handrail_indices = points_handrail_indices[1:]

            points_handrail = points_np[points_handrail_indices]

            detected_handrail = points_handrail[np.where(DBSCAN(eps=0.02, min_samples=1).fit(points_handrail).labels_ == 0)]
            stop = timeit.default_timer()
            rospy.loginfo("~~~~~~~~~~~~~Pointcloud took " + str(stop - start) + " seconds.~~~~~~~~~~~~~~~~")
            detected_handrail_o3d = o3d.geometry.PointCloud()
            detected_handrail_o3d.points = o3d.utility.Vector3dVector(detected_handrail)
            start = timeit.default_timer()
            reg_p2p = execute_global_registration(self.registered_handrail_o3d, detected_handrail_o3d)
            stop = timeit.default_timer()
            rospy.loginfo("~~~~~~~~~~~~~Registration took " + str(stop - start) + " seconds.~~~~~~~~~~~~~~~~")


            self.imgMsg = None
            self.pcMsg = None
            self.busy = False
        return True




if __name__=="__main__":
    # Initialize node
    rospy.init_node("handrail_pose_estimator_node")

    # Define detector object
    dm = HandrailDetectorManager()

    # Spin
    rospy.spin()
