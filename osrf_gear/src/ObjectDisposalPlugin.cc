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
GZ_REGISTER_MODEL_PLUGIN(ObjectDisposalPlugin)

/////////////////////////////////////////////////
ObjectDisposalPlugin::ObjectDisposalPlugin() : SideContactPlugin()
{
}

/////////////////////////////////////////////////
ObjectDisposalPlugin::~ObjectDisposalPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  SideContactPlugin::Load(_model, _sdf);
  this->centerOfGravityCheck = false;
  if (_sdf->HasElement("center_of_gravity_check"))
  {
    this->centerOfGravityCheck = _sdf->Get<bool>("center_of_gravity_check");
  }
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  this->CalculateContactingModels();
  this->ActOnContactingModels();
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::ActOnContactingModels()
{
  // Only remove models if their center of gravity is "above" the link
  // TODO: make more general than just z axis
  auto linkBox = this->parentLink->GetBoundingBox();
  auto linkBoxMax = linkBox.max;
  auto linkBoxMin = linkBox.min;
  linkBoxMin.z = std::numeric_limits<double>::lowest();
  linkBoxMax.z = std::numeric_limits<double>::max();
  auto disposalBox = math::Box(linkBoxMin, linkBoxMax);

  for (auto model : this->contactingModels) {
    if (model) {
      bool removeModel = true;
      if (this->centerOfGravityCheck)
      {
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
        removeModel = disposalBox.Contains(modelCog);
      }
      if (removeModel)
      {
        gzdbg << "[" << this->model->GetName() << "] Removing model: " << model->GetName() << "\n";
        this->world->RemoveModel(model);
      }
    }
  }
}
