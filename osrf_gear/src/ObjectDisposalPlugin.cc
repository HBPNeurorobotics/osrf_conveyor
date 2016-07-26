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

#include <limits>
#include <string>

#include "ObjectDisposalPlugin.hh"
#include <gazebo/transport/Node.hh>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ObjectDisposalPlugin)

/////////////////////////////////////////////////
ObjectDisposalPlugin::ObjectDisposalPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ObjectDisposalPlugin::~ObjectDisposalPlugin()
{
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ObjectDisposalPlugin requires a ContactSensor.\n";
    return;
  }

  std::string worldName = this->parentSensor->WorldName();
  this->world = physics::get_world(worldName);
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(worldName);

  std::string linkName = this->parentSensor->ParentName();
  this->link = boost::dynamic_pointer_cast<physics::Link>(this->world->GetEntity(linkName));

  std::string defaultCollisionName = linkName + "::__default__";
  if (this->parentSensor->GetCollisionCount() != 1 ||
        this->parentSensor->GetCollisionName(0) == defaultCollisionName)
  {
    gzerr << "ObjectDisposalPlugin requires a single collision to observe contacts for\n";
    return;
  }

  this->collisionName = this->parentSensor->GetCollisionName(0);
  gzdbg << "Watching contacts on collision: " << this->collisionName << "\n";

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ObjectDisposalPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::OnUpdate()
{
  this->CalculateContactingModels();
  this->ActOnContactingModels();

}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::CalculateContactingModels()
{
  auto linkPose = this->link->GetWorldPose().Ign();
  math::Vector3 linkTopNormal = linkPose.Rot().RotateVector(ignition::math::Vector3d::UnitZ);

  // Get all the contacts
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  this->contactingModels.clear();
  double factor = 1.0;

  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    // Get the collision that's not the link
    std::string collision = contacts.contact(i).collision1();
    if (this->collisionName == contacts.contact(i).collision1()) {
      collision = contacts.contact(i).collision2();
      factor = -1.0; // the frames are reversed for the alignment check
    }

    // Only consider models ontop of the link (collision normal aligned with +z of link)
    for (int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      ignition::math::Vector3d contactNormal = msgs::ConvertIgn(contacts.contact(i).normal(j));
      double alignment = factor * linkTopNormal.Dot(contactNormal);
      if (alignment > 0.0) {
        physics::CollisionPtr collisionPtr =
          boost::dynamic_pointer_cast<physics::Collision>(this->world->GetEntity(collision));
        if (collisionPtr) { // ensure the collision hasn't been deleted
          physics::ModelPtr model = collisionPtr->GetLink()->GetModel();
          this->contactingModels.insert(model);
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::ActOnContactingModels()
{
  // Only remove models if their center of gravity is "above" the link
  auto linkBox = this->link->GetBoundingBox();
  auto linkBoxMax = linkBox.max;
  auto linkBoxMin = linkBox.min;
  linkBoxMin.z = std::numeric_limits<double>::lowest();
  linkBoxMax.z = std::numeric_limits<double>::max();
  auto disposalBox = math::Box(linkBoxMin, linkBoxMax);

  for (auto model : this->contactingModels) {
    if (model) {
      // Calculate the center of gravity of the model
      math::Vector3 modelCog = math::Vector3::Zero;
      double modelMass = 0.0;
      for (auto modelLink : model->GetLinks())
      {
        double linkMass = modelLink->GetInertial()->GetMass();
        modelCog += modelLink->GetWorldCoGPose().pos * linkMass;
        modelMass += linkMass;
      }
      if (modelMass > 0.0)
      {
        modelCog /= modelMass;
      }
      if (disposalBox.Contains(modelCog))
      {
        gzdbg << "Removing model: " << model->GetName() << "\n";
        this->world->RemoveModel(model);
      }
    }
  }
}
