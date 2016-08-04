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
  this->trayID = this->model->GetName();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosNode = new ros::NodeHandle("");
  this->currentKitPub = this->rosNode->advertise<osrf_gear::Kit>(
    "/ariac/trays", 1000);
}

/////////////////////////////////////////////////
void KitTrayPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  if (!this->newMsg)
  {
    return;
  }

  auto prevNumberContactingModels = this->contactingModels.size();
  this->CalculateContactingModels();
  if (prevNumberContactingModels != this->contactingModels.size()) {
    gzdbg << this->parentSensor->Name() << ": number of contacting models: " \
      << this->contactingModels.size() << "\n";
  }
  this->ProcessContactingModels();
  this->PublishKitMsg();
}

/////////////////////////////////////////////////
void KitTrayPlugin::ProcessContactingModels()
{
  this->currentKit.objects.clear();
  auto trayPose = this->model->GetWorldPose().Ign();
  for (auto model : this->contactingModels) {
    if (model) {
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
void KitTrayPlugin::PublishKitMsg()
{
  // Publish current kit
  osrf_gear::Kit msgKit;
  msgKit.tray.data = this->trayID;
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
    msgKit.objects.push_back(msgObj);
  }
  this->currentKitPub.publish(msgKit);
}
