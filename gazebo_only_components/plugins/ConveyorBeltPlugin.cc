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

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ConveyorBeltPlugin)

/////////////////////////////////////////////////
ConveyorBeltPlugin::ConveyorBeltPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ConveyorBeltPlugin::~ConveyorBeltPlugin()
{
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

  this->beltCollisionName = this->parentSensor->Name();

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ConveyorBeltPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  gzdbg << "Contacts detected: " << contacts.contact_size() << "\n";

  std::string worldName = this->parentSensor->WorldName();
  physics::WorldPtr world = physics::get_world(worldName);

  this->contactingLinkPtrs.clear();
  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    // Get the collision that's not the belt (doesn't seem to be a standard order)
    std::string collision = contacts.contact(i).collision1();
    if (this->beltCollisionName == contacts.contact(i).collision1()) {
      collision = contacts.contact(i).collision2();
    }
      
    // Only act on objects on top of belt
    for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      if (contacts.contact(i).position(j).z() > 0.05) {
        physics::CollisionPtr collisionPtr = boost::dynamic_pointer_cast<physics::Collision>(world->GetEntity(collision));
        physics::LinkPtr linkPtr = collisionPtr->GetLink();
        this->contactingLinkPtrs.insert(linkPtr);
      }
    }
  }
  gzdbg << "Number of links in collision with belt: " << this->contactingLinkPtrs.size() << "\n";
  for (auto linkPtr : contactingLinkPtrs) {
    std::cout << "Collision with: " << linkPtr->GetScopedName() << "\n";
    linkPtr->SetLinearVel(math::Vector3(0, 0.5, 0));
  }

}
