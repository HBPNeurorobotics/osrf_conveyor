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
void ConveyorBeltPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
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

  std::string beltLinkName = this->parentSensor->ParentName();
  this->beltLink =
    boost::dynamic_pointer_cast<physics::Link>(this->world->GetEntity(beltLinkName));
  this->beltCollisionName = this->parentSensor->GetCollisionName(0); // assuming only one collision which is belt....

  std::string controlCommandTopic = this->Topic("control_command");
  gzdbg << "Subscribing to control commands on topic: " << controlCommandTopic << "\n";
  this->controlCommandSub = this->node->Subscribe(controlCommandTopic,
      &ConveyorBeltPlugin::OnControlCommand, this);

  std::lock_guard<std::mutex> lock(this->stateMutex);
  this->state = false;

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
  std::lock_guard<std::mutex> lock(this->stateMutex);
  bool state = this->state;
  this->ActOnContactingLinks(state);

}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::CalculateContactingLinks()
{
  double beltHeight = this->beltLink->GetBoundingBox().max.z;

  // Get all the contacts
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();

  this->contactingLinkPtrs.clear();
  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    // Get the collision that's not the belt
    std::string collision = contacts.contact(i).collision1();
    if (this->beltCollisionName == contacts.contact(i).collision1()) {
      collision = contacts.contact(i).collision2();
    }

    // Only consider links ontop of belt
    for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      if (contacts.contact(i).position(j).z() > (beltHeight - 0.001)) {
        physics::CollisionPtr collisionPtr =
          boost::dynamic_pointer_cast<physics::Collision>(this->world->GetEntity(collision));
        collisionPtr->GetBoundingBox();
        physics::LinkPtr linkPtr = collisionPtr->GetLink();
        this->contactingLinkPtrs.insert(linkPtr);
      }
    }
  }
  gzdbg << "Number of links ontop of belt: " << this->contactingLinkPtrs.size() << "\n";
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::ActOnContactingLinks(bool state)
{
  for (auto linkPtr : this->contactingLinkPtrs) {
    linkPtr->SetLinearVel(math::Vector3(0, state? 0.5 : 0, 0));
  }
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::OnControlCommand(ConstHeaderPtr& _msg)
{
  gzdbg << "Received control command of: " << _msg->index() << "\n";
  std::lock_guard<std::mutex> lock(this->stateMutex);
  this->state = _msg->index();
}
