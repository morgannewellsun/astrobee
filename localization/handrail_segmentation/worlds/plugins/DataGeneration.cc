
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
#include <random>

IGNITION_ADD_PLUGIN(
    data_generation::DataGeneration,
    ignition::gazebo::System,
    data_generation::DataGeneration::ISystemConfigure,
    data_generation::DataGeneration::ISystemPreUpdate)

std::default_random_engine re;
using namespace data_generation;

//////////////////////////////////////////////////
DataGeneration::DataGeneration()
{
}

//////////////////////////////////////////////////
DataGeneration::~DataGeneration()
{
}



double fRand(double fMax)
{
    std::uniform_real_distribution<double> unif(-fMax,fMax);
    return unif(re);
}


ignition::gazebo::components::Pose generateError(ignition::math::Pose3d currentPose)
{
  double tEpsilon = 0.15;
  double aEpsilon = 0.07;
  return ignition::gazebo::components::Pose(
                          ignition::math::Pose3d(currentPose.Pos().X() + fRand(tEpsilon), currentPose.Pos().Y() + fRand(tEpsilon), currentPose.Pos().Z() + fRand(tEpsilon),
                          currentPose.Rot().Euler().X() + fRand(aEpsilon), currentPose.Rot().Euler().Y() + fRand(aEpsilon), currentPose.Rot().Euler().Z() + fRand(aEpsilon)));

}


//////////////////////////////////////////////////
void DataGeneration::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr)
{
    this->entity = _entity;
    this->entityCreator = ignition::gazebo::SdfEntityCreator(_ecm, _eventMgr);

     this->handrailInspectPositions.push_back(ignition::math::Pose3d(4.01, 0.5, 4.6, 0, 1.57, 0)); // <model name="handrail 8.5 (1)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(4.01, -0.5, 9.6, 0, 1.57, 0)); // <model name="handrail 21.5 (1)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(5.02, 0, 15.0, 0, 1.57, 0)); // <model name="handrail 41.5 (1)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(6.09, 0.5, 19.6, 0, 1.57, 0)); //<model name="handrail 30 (1)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(6.09, 0.5, 24.8, 0, 1.57, 0)); // <model name="handrail 8.5 (2)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(2.94, 0.4, 29.6, 0, 1.57, 0)); // <model name="handrail 8.5 (3)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(2.94, -0.4, 34.6, 0, 1.57, 0)); // <model name="handrail 8.5 (4)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(1.3, -0.5, 40.0, 0, 1.57, 0)); // <model name="handrail 41.5 (2)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(1.3, 0, 45.0, 0, 1.57, 0)); // <model name="handrail 41.5 (3)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(2.94, 0, 50, 0, 0, -1.57)); // <model name="handrail 41.5 (4)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(6.09, 0, 55.6, 0, 0, -1.57)); // <model name="handrail 21.5 (2)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(6.09, 0, 59.6, 0, 0, -1.57)); // <model name="handrail 8.5 (5)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(5.02, 0, 64.6, 0, 0, -1.57)); // <model name="handrail 30 (2)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(5.02, 0, 70.65, 0, 0, -1.57)); // <model name="handrail 21.5 (3)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(7.16, 0, 74.6, 0, 0, -1.57)); // <model name="handrail 30 (3)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(7.16, 0, 80.5, 0, 0, -1.57)); // <model name="handrail 8.5 (6)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(7.16, 0, 85.8, 0, 0, -1.57)); // <model name="handrail 8.5 (7)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(2.94, 0, 90, 0, 0, 1.57)); // <model name="handrail 30 (4)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(1.87, 0, 95.5, 0, 0, 1.57)); // <model name="handrail 30 (5)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(7.16, 0, 100.4, 0, 0, 1.57)); // <model name="handrail 30 (6)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(7.16, 0, 104.5, 0, 0, 1.57)); // <model name="handrail 21.5 (4)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(5.02, 0, 109.8, 0, 0, 1.57)); // <model name="handrail 21.5 (5)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(5.02, 0, 115.6, 0, 0, 1.57)); // <model name="handrail 21.5 (6)">
     this->handrailInspectPositions.push_back(ignition::math::Pose3d(1.87, 0, 119.43, 0, 0, 1.57)); // <model name="handrail 21.5 (7)">
}

//////////////////////////////////////////////////
void DataGeneration::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm){

      auto sec = std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count();
      if(sec > this->lastPositionChange && this->n_count < handrailInspectPositions.size() * this->NUM_IMAGES_EACH){
        auto poseComp = _ecm.Component<ignition::gazebo::components::Pose>(this->entity);


        int handrailId = int(this->n_count/NUM_IMAGES_EACH);
        *poseComp = generateError(this->handrailInspectPositions[handrailId]);


        _ecm.SetChanged(this->entity, ignition::gazebo::components::Pose::typeId,
          ignition::gazebo::ComponentState::OneTimeChange);
        this->lastPositionChange = sec;
        this->n_count += 1;
        if (int(this->n_count/NUM_IMAGES_EACH) > handrailId){
          std::cout << "Handrail number " << handrailId << " is completed. \n";
        }
      }

      if(this->n_count >= handrailInspectPositions.size() * this->NUM_IMAGES_EACH){
        this->entityCreator.RequestRemoveEntity(this->entity);
      }

}
