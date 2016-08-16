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
}

/////////////////////////////////////////////////
ROSAGVPlugin::~ROSAGVPlugin()
{
  this->rosnode->shutdown();
}

/////////////////////////////////////////////////
void ROSAGVPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
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

  std::string topic = "/sim/agv";
  if (_sdf->HasElement("topic"))
    topic = _sdf->Get<std::string>("topic");

  this->rosnode = new ros::NodeHandle(this->robotNamespace);

  // start: 0.3 3.3 0 0 3.1415
  // 1: 1.5707
  // 2: -4.2 3.3 0 0 0 0
  // 3: 1.5707
  // 4: -4.2 9.45 0 0 0

  this->anim.reset(
      new gazebo::common::PoseAnimation("agv1", 22, false));

  gazebo::common::PoseKeyFrame *key;
  key = anim->CreateKeyFrame(0);
  key->Translation(ignition::math::Vector3d(0.3, 3.3, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 3.1415));

  /*key = anim->CreateKeyFrame(2);
  key->Translation(ignition::math::Vector3d(0.3, 3.3, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));
  */

  key = anim->CreateKeyFrame(4);
  key->Translation(ignition::math::Vector3d(-4.2, 3.3, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 3.1415));

  /*key = anim->CreateKeyFrame(6);
  key->Translation(ignition::math::Vector3d(-4.2, 3.3, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 0));
  */

  key = anim->CreateKeyFrame(10);
  key->Translation(ignition::math::Vector3d(-4.2, 9.45, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 3.1415));

  // Turn around
  /*key = anim->CreateKeyFrame(12);
  key->Translation(ignition::math::Vector3d(-4.2, 9.45, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 3.1415));
  */

  key = anim->CreateKeyFrame(16);
  key->Translation(ignition::math::Vector3d(-4.2, 3.3, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 3.1415));

  /*key = anim->CreateKeyFrame(18);
  key->Translation(ignition::math::Vector3d(-4.2, 3.3, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, -1.5707));
  */

  /*key = anim->CreateKeyFrame(20);
  key->Translation(ignition::math::Vector3d(0.3, 3.3, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, -1.5707));
  */

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
  gzdbg << "AGV tray complete\n";
  if (_req.trayComplete)
    this->model->SetAnimation(anim);
  _res.success = true;
  return true;
}
