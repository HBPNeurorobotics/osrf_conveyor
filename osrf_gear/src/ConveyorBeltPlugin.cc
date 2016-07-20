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

#include <boost/algorithm/string/replace.hpp>
#include <string>

#include "ConveyorBeltPlugin.hh"
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ConveyorBeltPlugin)

/////////////////////////////////////////////////
ConveyorBeltPlugin::ConveyorBeltPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ConveyorBeltPlugin::~ConveyorBeltPlugin()
{
  this->parentSensor.reset();
  this->world.reset();
}

//////////////////////////////////////////////////
std::string ConveyorBeltPlugin::Topic(std::string topicName) const
{
  std::string globalTopicName = "~/";
  globalTopicName += this->parentSensor->Name() + "/" + this->GetHandle() + "/" + topicName;
  boost::replace_all(globalTopicName, "::", "/");

  return globalTopicName;
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ConveyorBeltPlugin requires a ContactSensor.\n";
    return;
  }

  std::string worldName = this->parentSensor->WorldName();
  this->world = physics::get_world(worldName);
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(worldName);

  if (_sdf->HasElement("belt_velocity"))
  {
    this->beltVelocity = _sdf->Get<double>("belt_velocity");
  }
  else {
    this->beltVelocity = 0.5;
  }
  gzdbg << "Using belt velocity of: " << this->beltVelocity << " m/s\n";

  std::string beltLinkName = this->parentSensor->ParentName();
  this->beltLink =
    boost::dynamic_pointer_cast<physics::Link>(this->world->GetEntity(beltLinkName));

  std::string defaultCollisionName = beltLinkName + "::__default__";
  if (this->parentSensor->GetCollisionCount() != 1 ||
        this->parentSensor->GetCollisionName(0) == defaultCollisionName)
  {
    gzerr << "ConveyorBeltPlugin requires a single collision to observe contacts for\n";
    return;
  }

  this->beltCollisionName = this->parentSensor->GetCollisionName(0);

  std::string controlCommandTopic;
  if (_sdf->HasElement("control_command_topic"))
  {
      controlCommandTopic = _sdf->Get<std::string>("control_command_topic");
  }
  else {
      controlCommandTopic = this->Topic("control_command");
  }
  this->controlCommandSub = this->node->Subscribe(controlCommandTopic,
      &ConveyorBeltPlugin::OnControlCommand, this);

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ConveyorBeltPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::OnUpdate()
{
  this->CalculateContactingLinks();
  std::lock_guard<std::mutex> lock(this->mutex);
  double velocity = this->beltVelocity;
  this->ActOnContactingLinks(velocity);

}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::CalculateContactingLinks()
{
  auto beltPose = this->beltLink->GetWorldPose().Ign();
  math::Vector3 beltTopNormal = beltPose.Rot().RotateVector(ignition::math::Vector3d::UnitZ);

  // Get all the contacts
  //FIXME: this does not accurately report the contacts if objects are stationary (when using ODE)
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();

  auto prevNumberContactingLinks = this->contactingLinks.size();
  this->contactingLinks.clear();
  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    // Get the collision that's not the belt
    std::string collision = contacts.contact(i).collision1();
    if (this->beltCollisionName == contacts.contact(i).collision1()) {
      collision = contacts.contact(i).collision2();
    }

    // Only consider links ontop of belt (collision normal aligned with +z of belt)
    for (int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      ignition::math::Vector3d contactNormal = msgs::ConvertIgn(contacts.contact(i).normal(j));
      double alignment = beltTopNormal.Dot(contactNormal);
      if (alignment > 0.0) {
        physics::CollisionPtr collisionPtr =
          boost::dynamic_pointer_cast<physics::Collision>(this->world->GetEntity(collision));
        if (collisionPtr) { // ensure the collision hasn't been deleted
          physics::LinkPtr linkPtr = collisionPtr->GetLink();
          this->contactingLinks.insert(linkPtr);
        }
      }
    }
  }
  if (prevNumberContactingLinks != this->contactingLinks.size()) {
    gzdbg << "Number of links ontop of belt: " << this->contactingLinks.size() << "\n";
  }
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::ActOnContactingLinks(double velocity)
{
  ignition::math::Vector3d velocity_beltFrame(0.0, velocity, 0.0);
  auto beltPose = this->beltLink->GetWorldPose().Ign();
  math::Vector3 velocity_worldFrame = beltPose.Rot().RotateVector(velocity_beltFrame);
  for (auto linkPtr : this->contactingLinks) {
    if (linkPtr) {
      linkPtr->SetLinearVel(velocity_worldFrame);
    }
  }
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::OnControlCommand(ConstHeaderPtr& _msg)
{
  double requestedVelocity = std::stod(_msg->str_id());
  gzdbg << "Received control command of: " << requestedVelocity << "\n";
  this->SetVelocity(requestedVelocity);
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::SetVelocity(double velocity)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  gzdbg << "Setting velocity to: " << velocity << "\n";
  this->beltVelocity = velocity;
}
