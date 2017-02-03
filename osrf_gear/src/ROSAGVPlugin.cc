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
#include "ROSAGVPlugin.hh"

#include <gazebo/common/common.hh>
#include <ignition/math.hh>
#include <osrf_gear/SubmitTray.h>
#include <std_srvs/Trigger.h>

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
  std::string index;

  if (_sdf->HasElement("index"))
  {
    index = _sdf->Get<std::string>("index");
  }
  else
  {
    gzerr << "AGV is missing an index. The AGV will not work.\n";
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

  this->agvName = std::string("agv") + index;

  std::string agvControlTopic = "/ariac/" + this->agvName;
  ROS_DEBUG_STREAM("Using AGV control service topic: " << agvControlTopic);

  std::string submitTrayTopic = "submit_tray";
  if (_sdf->HasElement("submit_tray_service_name"))
    submitTrayTopic = _sdf->Get<std::string>("submit_tray_service_name");
  ROS_DEBUG_STREAM("Using submit tray service topic: " << submitTrayTopic);

  std::string clearTrayServiceName = "clear_tray";
  if (_sdf->HasElement("clear_tray_service_name"))
    clearTrayServiceName = _sdf->Get<std::string>("clear_tray_service_name");
  ROS_DEBUG_STREAM("Using clear tray service topic: " << clearTrayServiceName);

  this->rosnode = new ros::NodeHandle(this->robotNamespace);

  this->anim.reset(new gazebo::common::PoseAnimation(this->agvName, 22, false));

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

  this->rosService = this->rosnode->advertiseService(agvControlTopic,
      &ROSAGVPlugin::OnCommand, this);

  // Client for submitting trays for inspection.
  this->rosSubmitTrayClient =
    this->rosnode->serviceClient<osrf_gear::SubmitTray>(submitTrayTopic);

  // Client for clearing trays.
  this->rosClearTrayClient =
    this->rosnode->serviceClient<std_srvs::Trigger>(clearTrayServiceName);
}

/////////////////////////////////////////////////
bool ROSAGVPlugin::OnCommand(
  osrf_gear::AGVControl::Request &_req,
  osrf_gear::AGVControl::Response &_res)
{
  bool triggerAnim = this->agvName == "agv1" &&
    (anim->GetTime() <= 0.0 || anim->GetTime() >= anim->GetLength());

  if (triggerAnim)
  {
    anim->SetTime(0);
    this->model->SetAnimation(anim);
  }

  if (!this->rosSubmitTrayClient.exists())
  {
    this->rosSubmitTrayClient.waitForExistence();
  }

  // Make a service call to submit the tray for inspection.
  osrf_gear::SubmitTray srv;
  srv.request.tray_id = this->agvName + "::kit_tray::kit_tray::tray";
  srv.request.kit_type = _req.kit_type;
  this->rosSubmitTrayClient.call(srv);
  if (!srv.response.success) {
    ROS_ERROR_STREAM("Failed to submit tray for inspection.");
    _res.success = false;
    return false;
  }
  ROS_INFO_STREAM("Result of inspection: " << srv.response.inspection_result);
  _res.success = true;

  if (!this->rosClearTrayClient.exists())
  {
    this->rosClearTrayClient.waitForExistence();
  }
  std_srvs::Trigger clear_srv;
  this->rosClearTrayClient.call(clear_srv);


  return true;
}
