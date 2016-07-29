/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <limits>
#include <string>

#include "KitTrayPlugin.hh"
#include <gazebo/transport/Node.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Quaternion.hh>
#include <osrf_gear/Goal.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(KitTrayPlugin)

/////////////////////////////////////////////////
KitTrayPlugin::KitTrayPlugin() : SideContactPlugin()
{
}

/////////////////////////////////////////////////
KitTrayPlugin::~KitTrayPlugin()
{
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void KitTrayPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

  SideContactPlugin::Load(_model, _sdf);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosNode = new ros::NodeHandle("");
  this->goalSub = this->rosNode->subscribe(
    "/ariac/goals", 10, &KitTrayPlugin::OnGoalReceived, this);
}

/////////////////////////////////////////////////
void KitTrayPlugin::OnUpdate()
{
  auto prevNumberContactingModels = this->contactingModels.size();
  this->CalculateContactingModels();
  if (prevNumberContactingModels != this->contactingModels.size()) {
    gzdbg << this->parentSensor->Name() << ": number of contacting models: " \
      << this->contactingModels.size() << "\n";
  }
  this->ActOnContactingModels();
}

/////////////////////////////////////////////////
void KitTrayPlugin::ActOnContactingModels()
{
  int score = 0;
  for (auto model : this->contactingModels) {
    if (model) {
      std::string modelName = model->GetName();
      gzdbg << "Checking model: " << modelName << "\n";
      for (auto kitObj : this->kit.objects)
      {
        if (modelName == kitObj.type)
        {
          score += 1;
          gzdbg << "Tray score: " << score << "\n";
          break;
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void KitTrayPlugin::OnGoalReceived(const osrf_gear::Goal::ConstPtr & goalMsg)
{
  gzdbg << "Assigned a kit to monitor\n";
  this->kit.objects.clear();
  // TODO: only pay attention to kits which 'belong' to this tray
  for (auto objMsg : goalMsg->kits.at(0).objects)
  {
    ariac::KitObject obj;
    obj.type = objMsg.type;
    geometry_msgs::Point p = objMsg.pose.position;
    geometry_msgs::Quaternion o = objMsg.pose.orientation;
    obj.pose = math::Pose(math::Vector3(p.x, p.y, p.z), math::Quaternion(o.x, o.y, o.z, o.w));
    this->kit.objects.push_back(obj);
  }
}
