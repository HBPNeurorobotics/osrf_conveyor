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

#include "SideContactPlugin.hh"
#include <ignition/math/Vector3.hh>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(SideContactPlugin)

/////////////////////////////////////////////////
SideContactPlugin::SideContactPlugin() : ModelPlugin()
{
}

/////////////////////////////////////////////////
SideContactPlugin::~SideContactPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void SideContactPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "Model pointer is null");

  if (!_sdf->HasElement("contact_sensor_name"))
  {
    gzerr << "'contact_sensor_name' not specified in SDF\n";
  }
  this->model = _model;
  this->world = this->model->GetWorld();
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());

  this->contactSensorName = _sdf->Get<std::string>("contact_sensor_name");
  bool sensorFound = this->FindContactSensor();
  if (!sensorFound || !this->parentSensor)
  {
    gzerr << "Contact sensor not found: " << this->contactSensorName << "\n";
  }

  std::string parentLinkName = this->parentLink->GetScopedName();
  std::string defaultCollisionName = parentLinkName + "::__default__";
  if (this->parentSensor->GetCollisionCount() != 1)
  {
    gzerr << "SideContactPlugin requires a single collision to observe contacts for\n";
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
  gzdbg << "[" << this->model->GetName() << "] Watching collisions on: " << this->collisionName << "\n";

  if (_sdf->HasElement("contact_side_normal"))
  {
    this->sideNormal = _sdf->Get<ignition::math::Vector3d>("contact_side_normal");
  }
  else
  {
    this->sideNormal = ignition::math::Vector3d(0, 0, 1);
  }


  // FIXME: how to not hard-code this gazebo prefix?
  std::string contactTopic = "/gazebo/" + this->scopedContactSensorName;
  boost::replace_all(contactTopic, "::", "/");
  this->contactSub =
    this->node->Subscribe(contactTopic, &SideContactPlugin::OnContactsReceived, this);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&SideContactPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
bool SideContactPlugin::FindContactSensor()
{
  auto sensorManager = sensors::SensorManager::Instance();
  auto links = this->model->GetLinks();
  for (const auto &link : links)
  {
    std::string scopedContactSensorName =
      this->world->GetName() + "::" + link->GetScopedName() + "::" + this->contactSensorName;
    for (unsigned int i = 0; i < link->GetSensorCount(); ++i)
    {
      if (link->GetSensorName(i) == scopedContactSensorName)
      {
        this->parentLink = link;
        this->scopedContactSensorName = scopedContactSensorName;
        this->parentSensor =
          std::static_pointer_cast<sensors::ContactSensor>(
            sensorManager->GetSensor(this->contactSensorName));
        return this->parentSensor != 0;
      }
    }
  }
  return false;
}

/////////////////////////////////////////////////
void SideContactPlugin::OnContactsReceived(ConstContactsPtr& _msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  this->newestContactsMsg = *_msg;
  this->newMsg = true;
}

/////////////////////////////////////////////////
void SideContactPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  this->CalculateContactingModels();
}

/////////////////////////////////////////////////
void SideContactPlugin::CalculateContactingLinks()
{
  if (!this->newMsg)
  {
    return;
  }

  auto parentLinkPose = this->parentLink->GetWorldPose().Ign();
  math::Vector3 parentLinkTopNormal = parentLinkPose.Rot().RotateVector(this->sideNormal);

  // Get all the contacts
  boost::mutex::scoped_lock lock(this->mutex);
  msgs::Contacts contacts = this->newestContactsMsg;
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

    // Only consider links with collision normals aligned with the normal of the link's side
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
  this->newMsg = false;
}

/////////////////////////////////////////////////
void SideContactPlugin::CalculateContactingModels()
{
  this->CalculateContactingLinks();
  this->contactingModels.clear();
  for (auto link : this->contactingLinks)
  {
    physics::ModelPtr model = link->GetModel();
    this->contactingModels.insert(model);
  }
}
