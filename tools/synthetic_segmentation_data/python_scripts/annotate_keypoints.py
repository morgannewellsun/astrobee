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


# Python imports
import argparse
from xml.etree import ElementTree

# Third party imports
import numpy as np
import pandas as pd

# Local imports
import transform


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config_file", type=str, required=True)
    parser.add_argument("--world_file", type=str, required=True)
    parser.add_argument("--output_dir", type=str, required=True)
    return parser.parse_args()


def main(config_file, world_file, output_dir):

    # Reading parameters and data
    config = read_config(config_file)
    intrinsics_matrix = read_camera_intrinsics(world_file)


def read_config(config_file_path):
    config_dict = {}
    with open(config_file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if line and not line.startswith('#'):
                key, value = line.split('=', 1)
                config_dict[key] = value
    return config_dict


def read_camera_intrinsics(world_file_path):

    tree = ElementTree.parse(world_file_path)
    root = tree.getroot()
    for sensor in root.iter('sensor'):
        if 'name' in sensor.attrib and sensor.attrib['name'] == 'segmentation_camera':

            camera = sensor.find('camera')
            if camera is None:
                raise RuntimeError("Error parsing world file: <camera> not found.")
            
            horizontal_fov = camera.find('horizontal_fov')
            if horizontal_fov is None:
                raise RuntimeError("Error parsing world file: <horizontal_fov> not found.")
            
            image = camera.find('image')
            if image is None:
                raise RuntimeError("Error parsing world file: <image> not found.")
            
            width = image.find('width')
            height = image.find('height')
            if (width is None) or (height is None):
                raise RuntimeError("Error parsing world file: <width> and/or <height> not found.")
            
            fov_x = float(horizontal_fov.text)  # radians assumed
            c_x = float(width.text) / 2
            c_y = float(height.text) / 2
            f = c_x / np.tan(fov_x / 2)
            return np.array([[f, 0, c_x], [0, f, c_y], [0, 0, 1]])

    raise RuntimeError("Error parsing world file: segmentation_camera not found.")





if __name__ == "__main__":

    args = parse_args()
    main(
        config_file=args.config_file, 
        world_file=args.world_file, 
        output_dir=args.output_dir)
    








    # Load data
    data_dir = "/usr/local/home/mnsun/large_files/data/handrail/synthetic/new_with_gt"
    handrail_poses_csv = "/usr/local/home/mnsun/ros_ws/astrobee/src/tools/synthetic_segmentation_data/inspection_poses/handrail.csv"
    df_ground_truth_poses = pd.read_csv(data_dir + "/groundTruthPoses.csv")
    df_inspection_poses = pd.read_csv(handrail_poses_csv)

    for column_label in df_inspection_poses.columns:
        if "pose_inspection" in column_label:
            del df_inspection_poses[column_label]
    df_merged = pd.merge(df_ground_truth_poses, df_inspection_poses, how="left", on="name")

    # TODO: Replace this once testing is done
    df_thirty = df_merged[df_merged["label"] == 140]
    keypoint_positions_local = [
        [ 6.187375e-02, -1.315000e-04,  3.598150e-01],
        [ 6.187375e-02,  1.550000e-05, -3.588850e-01]]
    # keypoint_positions_local = [
    #     [3.588850e-01, 1.600000e-05, 2.834375e-02],
    #     [-3.598150e-01, -1.302500e-04,  2.834375e-02]]


    for 







