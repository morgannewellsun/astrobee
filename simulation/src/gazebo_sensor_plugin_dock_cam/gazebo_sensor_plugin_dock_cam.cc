/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

// ROS includes
#include <ros/ros.h>

// Sensor plugin interface
#include <astrobee_gazebo/astrobee_gazebo.h>

// IMU Sensor message
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

// FSW includes
#include <config_reader/config_reader.h>

// STL includes
#include <string>

namespace gazebo {

class GazeboSensorPluginDockCam : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginDockCam() :
    FreeFlyerSensorPlugin("dock_cam", "dock_cam", true), rate_(0.0) {}

  ~GazeboSensorPluginDockCam() {
    if (update_) {
      #if GAZEBO_MAJOR_VERSION > 7
      update_.reset();
      #else
      sensor_->DisconnectUpdated(update_);
      #endif
    }
  }

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(ros::NodeHandle *nh,
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::WideAngleCameraSensor>(sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginDockCam requires a parent camera sensor.\n";
      return;
    }


    // Set image constants
    image_msg_.is_bigendian = false;
    image_msg_.header.frame_id = GetFrame();
    image_msg_.encoding = sensor_msgs::image_encodings::MONO8;

    // Create a publisher
    pub_img_ = nh->advertise<sensor_msgs::Image>(TOPIC_HARDWARE_DOCK_CAM, 1,
      boost::bind(&GazeboSensorPluginDockCam::ToggleCallback, this),
      boost::bind(&GazeboSensorPluginDockCam::ToggleCallback, this));
    pub_pose_ = nh->advertise<geometry_msgs::PoseStamped>(TOPIC_DOCK_CAM_SIM_POSE, 10);
    pub_info_ = nh->advertise<sensor_msgs::CameraInfo>(TOPIC_DOCK_CAM_SIM_INFO, 10);
    pub_debug_ = nh->advertise<std_msgs::String>("/hw/cam_dock/debug", 10);

    // Read configuration
    config_reader::ConfigReader config;
    config.AddFile("simulation/simulation.config");
    if (!config.ReadFiles()) {
      ROS_FATAL("Failed to read simulation config file.");
      return;
    }
    bool dos = true;
    if (!config.GetBool("disable_cameras_on_speedup", &dos))
      ROS_FATAL("Could not read the drawing_width parameter.");
    if (!config.GetReal("dock_cam_rate", &rate_))
      ROS_FATAL("Could not read the drawing_width parameter.");
    config.Close();

    // If we have a sped up simulation and we need to disable the camera
    double simulation_speed = 1.0;
    if (nh->getParam("/simulation_speed", simulation_speed))
      if (simulation_speed > 1.0 && dos) rate_ = 0.0;

    // Toggle if the camera is active or not
    ToggleCallback();
  }

  // Only send measurements when extrinsics are available
  void OnExtrinsicsReceived(ros::NodeHandle *nh) {
    // Connect to the camera update event.
    update_ = sensor_->ConnectUpdated(
      std::bind(&GazeboSensorPluginDockCam::UpdateCallback, this));
  }

  // Turn camera on or off based on topic subscription
  void ToggleCallback() {
    if (pub_img_.getNumSubscribers() > 0 && rate_ > 0) {
      sensor_->SetUpdateRate(rate_);
      sensor_->SetActive(true);
    } else {
      sensor_->SetUpdateRate(0.0001);
      sensor_->SetActive(false);
    }
  }

  // Called on each sensor update event
  void UpdateCallback() {
    ros::Time curr_time = ros::Time::now();

    // Publish the nav cam pose
    #if GAZEBO_MAJOR_VERSION > 7
    Eigen::Affine3d sensor_to_world = SensorToWorld(GetModel()->WorldPose(), sensor_->Pose());
    #else
    Eigen::Affine3d sensor_to_world = SensorToWorld(GetModel()->GetWorldPose(), sensor_->Pose());
    #endif
    pose_msg_.header.frame_id = GetFrame();
    pose_msg_.header.stamp = curr_time;  // it is very important to get the time right
    pose_msg_.pose.position.x = sensor_to_world.translation().x();
    pose_msg_.pose.position.y = sensor_to_world.translation().y();
    pose_msg_.pose.position.z = sensor_to_world.translation().z();
    Eigen::Quaterniond q(sensor_to_world.rotation());
    pose_msg_.pose.orientation.w = q.w();
    pose_msg_.pose.orientation.x = q.x();
    pose_msg_.pose.orientation.y = q.y();
    pose_msg_.pose.orientation.z = q.z();
    pub_pose_.publish(pose_msg_);

    // Publish the nav cam intrinsics
    info_msg_.header.frame_id = GetFrame();
    info_msg_.header.stamp = curr_time;  // it is very important to get the time right
    FillCameraInfo(sensor_->Camera(), info_msg_);  // fill in from the camera pointer
    pub_info_.publish(info_msg_);
    // Publish the nav cam image
    image_msg_.header.stamp  = curr_time;
    fillImage(image_msg_, sensor_msgs::image_encodings::RGB8, sensor_->ImageHeight(), sensor_->ImageWidth(),
       3*sensor_->ImageWidth(), reinterpret_cast<const void*>(sensor_->ImageData()));


    // image_msg_.height            = sensor_->ImageHeight();
    // image_msg_.width             = sensor_->ImageWidth();
    // image_msg_.step              = image_msg_.width;
    // image_msg_.data.resize(image_msg_.step * image_msg_.height);
    // const uint8_t* data_start = reinterpret_cast<const uint8_t*>(sensor_->ImageData());
    // std::copy(data_start, data_start + image_msg_.step * image_msg_.height,
    //           image_msg_.data.begin());
    pub_img_.publish(image_msg_);


    rendering::DistortionPtr distort = sensor_->Camera()->LensDistortion();
    std_msgs::String msg;
    std::stringstream ss;
    ss << distort->K1() << " " << distort->K2() << " " << distort->K3() << " " << distort->P1() << " " << distort->P2();
    msg.data = ss.str();

    pub_debug_.publish(msg);
  }

 private:
   sensor_msgs::Image image_msg_;
   geometry_msgs::PoseStamped pose_msg_;
   sensor_msgs::CameraInfo info_msg_;
   ros::Publisher pub_img_;
   ros::Publisher pub_pose_;
   ros::Publisher pub_info_;
  ros::Publisher pub_debug_;
  std::shared_ptr<sensors::WideAngleCameraSensor> sensor_;
  event::ConnectionPtr update_;
  double rate_;
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginDockCam)

}   // namespace gazebo
