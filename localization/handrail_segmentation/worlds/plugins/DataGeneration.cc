
/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "DataGeneration.hh"

#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>
#include <iostream>
#include <fstream>
#include <random>

#include "CSVReader.hh"

IGNITION_ADD_PLUGIN(
    data_generation::DataGeneration,
    ignition::gazebo::System,
    data_generation::DataGeneration::ISystemConfigure,
    data_generation::DataGeneration::ISystemPreUpdate)

std::default_random_engine re;
namespace data_generation {

//////////////////////////////////////////////////
DataGeneration::DataGeneration() {}

//////////////////////////////////////////////////
DataGeneration::~DataGeneration() {}

double fRand(double fMax) {
  std::uniform_real_distribution<double> unif(-fMax, fMax);
  return unif(re);
}

ignition::gazebo::components::Pose generateError(ignition::math::Pose3d currentPose) {
  double tEpsilon = 0.15;
  double aEpsilon = 0.07;
  return ignition::gazebo::components::Pose(ignition::math::Pose3d(
    currentPose.Pos().X() + fRand(tEpsilon), currentPose.Pos().Y() + fRand(tEpsilon),
    currentPose.Pos().Z() + fRand(tEpsilon), currentPose.Rot().Euler().X() + fRand(aEpsilon),
    currentPose.Rot().Euler().Y() + fRand(aEpsilon), currentPose.Rot().Euler().Z() + fRand(aEpsilon)));
}

//////////////////////////////////////////////////
void DataGeneration::Configure(const ignition::gazebo::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                               ignition::gazebo::EntityComponentManager& _ecm,
                               ignition::gazebo::EventManager& _eventMgr) {
  this->entity = _entity;
  this->entityCreator = ignition::gazebo::SdfEntityCreator(_ecm, _eventMgr);

  // Read and load inspection poses from CSV
  CSVReader reader;
  vector<string> columnLabels = {"pose_inspection_0", "pose_inspection_1", "pose_inspection_2",
                                 "pose_inspection_3", "pose_inspection_4", "pose_inspection_5"};
  vector<vector<string>> inspectionPoses = reader.readCSV(this->filepathInspectionPoses, columnLabels);
  for (auto& pose : inspectionPoses) {
    this->handrailInspectPositions.push_back(ignition::math::Pose3d(std::stof(pose[0]), std::stof(pose[1]),
                                                                    std::stof(pose[2]), std::stof(pose[3]),
                                                                    std::stof(pose[4]), std::stof(pose[5])));
  }
}

//////////////////////////////////////////////////
void DataGeneration::PreUpdate(const ignition::gazebo::UpdateInfo& _info,
                               ignition::gazebo::EntityComponentManager& _ecm) {
  auto sec = std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count();

  if (sec > this->lastPositionChange && this->n_count < handrailInspectPositions.size() * this->NUM_IMAGES_EACH) {
    auto poseComp = _ecm.Component<ignition::gazebo::components::Pose>(this->entity);

    int handrailId = static_cast<int>(this->n_count / NUM_IMAGES_EACH);
    auto inspectionPoseWithError = generateError(this->handrailInspectPositions[handrailId]);
    *poseComp = inspectionPoseWithError;

    // Append ground truth pose to file
    std::ofstream file(this->filepathGroundTruth, std::ios_base::app);
    file << inspectionPoseWithError.Data().Pos().X() << "," << inspectionPoseWithError.Data().Pos().Y() << ","
         << inspectionPoseWithError.Data().Pos().Z() << "," << inspectionPoseWithError.Data().Rot().Euler().X() << ","
         << inspectionPoseWithError.Data().Rot().Euler().Y() << "," << inspectionPoseWithError.Data().Rot().Euler().Z()
         << std::endl;
    file.close();

    _ecm.SetChanged(this->entity, ignition::gazebo::components::Pose::typeId,
                    ignition::gazebo::ComponentState::OneTimeChange);
    this->lastPositionChange = sec;
    this->n_count += 1;
    if (static_cast<int>(this->n_count / NUM_IMAGES_EACH) > handrailId) {
      std::cout << "Handrail number " << handrailId << " is completed. \n";
    }
  }

  if (this->n_count >= handrailInspectPositions.size() * this->NUM_IMAGES_EACH) {
    this->entityCreator.RequestRemoveEntity(this->entity);
  }
}

}  // namespace data_generation
