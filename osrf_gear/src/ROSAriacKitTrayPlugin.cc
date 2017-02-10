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

#include <cstdlib>
#include <string>

#include <osrf_gear/KitTray.h>

#include "ROSAriacKitTrayPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(KitTrayPlugin)

/////////////////////////////////////////////////
KitTrayPlugin::KitTrayPlugin() : SideContactPlugin()
{
}

/////////////////////////////////////////////////
KitTrayPlugin::~KitTrayPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void KitTrayPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  SideContactPlugin::Load(_model, _sdf);

  if (this->updateRate > 0)
    gzdbg << "KitTrayPlugin running at " << this->updateRate << " Hz\n";
  else
    gzdbg << "KitTrayPlugin running at the default update rate\n";

  this->trayID = this->parentLink->GetScopedName();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosNode = new ros::NodeHandle("");
  this->currentKitPub = this->rosNode->advertise<osrf_gear::KitTray>(
    "/ariac/trays", 1000, boost::bind(&KitTrayPlugin::OnSubscriberConnect, this, _1));
  this->publishingEnabled = true;

  // Service for locking models to the tray
  std::string lockModelsServiceName = "lock_models";
  if (_sdf->HasElement("lock_models_service_name"))
    lockModelsServiceName = _sdf->Get<std::string>("lock_models_service_name");
  this->lockModelsServer =
    this->rosNode->advertiseService(lockModelsServiceName, &KitTrayPlugin::HandleLockModelsService, this);

  // Service for clearing the tray
  std::string clearServiceName = "clear";
  if (_sdf->HasElement("clear_tray_service_name"))
    clearServiceName = _sdf->Get<std::string>("clear_tray_service_name");
  this->clearTrayServer =
    this->rosNode->advertiseService(clearServiceName, &KitTrayPlugin::HandleClearService, this);
}

/////////////////////////////////////////////////
void KitTrayPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  // If we're using a custom update rate value we have to check if it's time to
  // update the plugin or not.
  if (!this->TimeToExecute())
  {
    return;
  }

  if (!this->newMsg)
  {
    return;
  }

  auto prevNumberContactingModels = this->contactingModels.size();
  this->CalculateContactingModels();
  if (prevNumberContactingModels != this->contactingModels.size()) {
    ROS_DEBUG_STREAM(this->parentLink->GetScopedName() << ": number of contacting models: "
      << this->contactingModels.size());
  }
  this->ProcessContactingModels();
  if (this->publishingEnabled)
  {
    this->PublishKitMsg();
  }
}

/////////////////////////////////////////////////
void KitTrayPlugin::ProcessContactingModels()
{
  // Make sure that models fixed to the tray are included in the contacting models,
  // even if they aren't contacting the tray anymore.
  for (auto fixedJoint : this->fixedJoints)
  {
    auto link = fixedJoint->GetChild();
    this->contactingLinks.insert(link);
    this->contactingModels.insert(link->GetParentModel());
  }
  this->currentKit.objects.clear();
  auto trayPose = this->parentLink->GetWorldPose().Ign();
  for (auto model : this->contactingModels) {
    if (model) {
      model->SetAutoDisable(false);
      ariac::KitObject object;

      // Determine the object type
      object.type = ariac::DetermineModelType(model->GetName());

      // Determine the pose of the object in the frame of the tray
      math::Pose objectPose = model->GetWorldPose();
      ignition::math::Matrix4d transMat(trayPose);
      ignition::math::Matrix4d objectPoseMat(objectPose.Ign());
      object.pose = (transMat.Inverse() * objectPoseMat).Pose();

      this->currentKit.objects.push_back(object);
    }
  }
}

/////////////////////////////////////////////////
void KitTrayPlugin::OnSubscriberConnect(const ros::SingleSubscriberPublisher& pub)
{
  auto subscriberName = pub.getSubscriberName();
  gzdbg << this->trayID << ": New subscription from node: " << subscriberName << std::endl;

  // During the competition, this environment variable will be set.
  auto compRunning = std::getenv("ARIAC_COMPETITION");
  if (compRunning && subscriberName.compare("/gazebo") != 0)
  {
    std::string errStr = "Competition is running so subscribing to this topic is not permitted.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    // Disable publishing of kit messages.
    // This will break the scoring but ensure competitors can't cheat.
    this->publishingEnabled = false;
  }
}

/////////////////////////////////////////////////
void KitTrayPlugin::PublishKitMsg()
{
  // Publish current kit
  osrf_gear::KitTray kitTrayMsg;
  kitTrayMsg.tray = this->trayID;
  for (const auto &obj : this->currentKit.objects)
  {
    osrf_gear::KitObject msgObj;
    msgObj.type = obj.type;
    msgObj.pose.position.x = obj.pose.pos.x;
    msgObj.pose.position.y = obj.pose.pos.y;
    msgObj.pose.position.z = obj.pose.pos.z;
    msgObj.pose.orientation.x = obj.pose.rot.x;
    msgObj.pose.orientation.y = obj.pose.rot.y;
    msgObj.pose.orientation.z = obj.pose.rot.z;
    msgObj.pose.orientation.w = obj.pose.rot.w;

    // Add the object to the kit.
    kitTrayMsg.kit.objects.push_back(msgObj);
  }
  this->currentKitPub.publish(kitTrayMsg);
}

/////////////////////////////////////////////////
void KitTrayPlugin::UnlockContactingModels()
{
  boost::mutex::scoped_lock lock(this->mutex);
  physics::JointPtr fixedJoint;
  for (auto fixedJoint : this->fixedJoints)
  {
    fixedJoint->Detach();
  }
  this->fixedJoints.clear();
}

/////////////////////////////////////////////////
void KitTrayPlugin::LockContactingModels()
{
  boost::mutex::scoped_lock lock(this->mutex);
  physics::JointPtr fixedJoint;
  for (auto model : this->contactingModels)
  {
  // Create the joint that will attach the models
  fixedJoint = this->world->GetPhysicsEngine()->CreateJoint(
        "fixed", this->model);
        std::cout << this->parentLink->GetName() << std::endl;
        std::cout << this->model->GetName() << std::endl;
  auto jointName = this->model->GetName() + "_" + model->GetName() + "__joint__";
  gzdbg << "Creating fixed joint: " << jointName << std::endl;
  fixedJoint->SetName(jointName);
  /*
  std::ostringstream newJointStr;
  newJointStr <<
    "<sdf version='" << "1.6" << "'>\n"
    "    <joint name=" << jointName << " type='fixed'>\n"
    "    <parent>" << this->parentLink->GetName() << "</parent>\n"
    "    <child>" << model->GetName() << "::link</child>\n"
    "  </joint>\n"
    "</sdf>\n";
    sdf::Element jointSdf;
    std::cout << newJointStr.str() << std::endl;
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("joint.sdf", sdf);
    sdf::readString(newJointStr.str(), sdf);
    fixedJoint->Load(sdf);
    std::cout << fixedJoint->GetChild()->GetScopedName() << std::endl;
    */

  model->SetGravityMode(false);
  model->GetLink(model->GetName()+"::link")->SetGravityMode(false);
  model->SetWorldPose(model->GetWorldPose() + math::Pose(0,0,0.01,0,0,0));
  auto link = model->GetLink(model->GetName() + "::link");
  if (link == NULL)
  {
    gzwarn << "Couldn't find link to make joint with";
    continue;
  }
    std::cout << link->GetScopedName() << std::endl;
    fixedJoint->Load(link, this->parentLink, math::Pose());
    fixedJoint->Attach(this->parentLink, link);
    fixedJoint->Init();
    this->fixedJoints.push_back(fixedJoint);
  model->SetAutoDisable(true);
  }
}

/////////////////////////////////////////////////
bool KitTrayPlugin::HandleLockModelsService(
  std_srvs::Trigger::Request & req,
  std_srvs::Trigger::Response & res)
{
  (void)req;
  this->LockContactingModels();
  res.success = true;
  return true;
}

/////////////////////////////////////////////////
bool KitTrayPlugin::HandleClearService(
  std_srvs::Trigger::Request & req,
  std_srvs::Trigger::Response & res)
{
  (void)req;
  this->UnlockContactingModels();
  this->ClearContactingModels();
  res.success = true;
  return true;
}
