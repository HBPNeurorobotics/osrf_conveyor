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

#include <string>

#include "TopContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(TopContactPlugin)

/////////////////////////////////////////////////
TopContactPlugin::TopContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
TopContactPlugin::~TopContactPlugin()
{
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void TopContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "TopContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&TopContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  std::string worldName = this->parentSensor->WorldName();
  this->world = physics::get_world(worldName);

  std::string parentLinkName = this->parentSensor->ParentName();
  this->parentLink =
    boost::dynamic_pointer_cast<physics::Link>(this->world->GetEntity(parentLinkName));

  std::string defaultCollisionName = parentLinkName + "::__default__";
  if (this->parentSensor->GetCollisionCount() != 1)
  {
    gzerr << "TopContactPlugin requires a single collision to observe contacts for\n";
    return;
  }

  this->collisionName = this->parentSensor->GetCollisionName(0);
  if (this->collisionName == defaultCollisionName)
  {
    // Use the first collision of the parent link by default
    if (this->parentLink->GetCollisions().empty())
    {
      gzerr << "Couldn't find any collisions for the contact sensor.";
      return;
    }
    unsigned int index = 0;
    this->collisionName = this->parentLink->GetCollision(index)->GetScopedName();
  }
}

/////////////////////////////////////////////////
void TopContactPlugin::OnUpdate()
{
  this->CalculateContactingModels();
}

/////////////////////////////////////////////////
void TopContactPlugin::CalculateContactingLinks()
{
  auto parentLinkPose = this->parentLink->GetWorldPose().Ign();
  math::Vector3 parentLinkTopNormal =
    parentLinkPose.Rot().RotateVector(ignition::math::Vector3d::UnitZ);

  // Get all the contacts
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  gzdbg << this->parentSensor->GetName() << " " << contacts.contact_size()<<"\n";
  this->contactingLinks.clear();
  double factor = 1.0;

  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    // Get the collision that's not the parent link
    std::string collision = contacts.contact(i).collision1();
    if (this->collisionName == contacts.contact(i).collision1()) {
      collision = contacts.contact(i).collision2();
      factor = -1.0; // the frames are reversed for the alignment check
    }

    // Only consider models ontop of the parent link (collision normal aligned with +z of link)
    for (int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      ignition::math::Vector3d contactNormal = msgs::ConvertIgn(contacts.contact(i).normal(j));
      double alignment = factor * parentLinkTopNormal.Dot(contactNormal);
      if (alignment > 0.0) {
        physics::CollisionPtr collisionPtr =
          boost::dynamic_pointer_cast<physics::Collision>(this->world->GetEntity(collision));
        if (collisionPtr) { // ensure the collision hasn't been deleted
          physics::LinkPtr link = collisionPtr->GetLink();
          this->contactingLinks.insert(link);
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void TopContactPlugin::CalculateContactingModels()
{
  this->CalculateContactingLinks();
  this->contactingModels.clear();
  for (auto link : this->contactingLinks)
  {
    physics::ModelPtr model = link->GetModel();
    this->contactingModels.insert(model);
  }
}
