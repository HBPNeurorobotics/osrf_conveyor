/*
 * Copyright 2016 Open Source Robotics Foundation
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
#include <gazebo/common/common.hh>
#include <ignition/math.hh>
#include "ROSAGVPlugin.hh"

#include <string>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSAGVPlugin);

/////////////////////////////////////////////////
ROSAGVPlugin::ROSAGVPlugin()
{
  this->index = 0;
}

/////////////////////////////////////////////////
ROSAGVPlugin::~ROSAGVPlugin()
{
  this->rosnode->shutdown();
}

/////////////////////////////////////////////////
void ROSAGVPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  _sdf->PrintValues("     ");
  if (_sdf->HasElement("index"))
  {
    this->index = _sdf->Get<int>("index");
  }

  // load parameters
  this->robotNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace = _sdf->GetElement(
        "robotNamespace")->Get<std::string>() + "/";
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  std::string topic = "/ariac/agv";
  if (_sdf->HasElement("topic"))
    topic = _sdf->Get<std::string>("topic");

  this->rosnode = new ros::NodeHandle(this->robotNamespace);

  this->anim.reset(new gazebo::common::PoseAnimation("agv", 22, false));

  gazebo::common::PoseKeyFrame *key = anim->CreateKeyFrame(0);
  key->Translation(ignition::math::Vector3d(0.3, 3.3, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 3.1415));

  key = anim->CreateKeyFrame(4);
  key->Translation(ignition::math::Vector3d(-4.2, 3.8, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 3.1415));

  key = anim->CreateKeyFrame(10);
  key->Translation(ignition::math::Vector3d(-4.2, 9.45, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 3.1415));

  key = anim->CreateKeyFrame(16);
  key->Translation(ignition::math::Vector3d(-4.2, 3.8, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 3.1415));

  key = anim->CreateKeyFrame(22);
  key->Translation(ignition::math::Vector3d(0.3, 3.3, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 3.1415));

  this->model = _parent;

  this->rosService = this->rosnode->advertiseService(topic,
      &ROSAGVPlugin::OnCommand, this);
}

/////////////////////////////////////////////////
bool ROSAGVPlugin::OnCommand(
  osrf_gear::AGVControl::Request &_req,
  osrf_gear::AGVControl::Response &_res)
{
  _res.success = _req.index == this->index && _req.trayComplete &&
      (anim->GetTime() <= 0.0 || anim->GetTime() >= anim->GetLength());

  if (_res.success)
  {
    anim->SetTime(0);
    this->model->SetAnimation(anim);
  }

  return _res.success;
}
