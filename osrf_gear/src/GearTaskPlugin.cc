/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <algorithm>
#include <string>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>

#include "osrf_gear//GearTaskPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(GearTaskPlugin)

/////////////////////////////////////////////////
GearTaskPlugin::GearTaskPlugin()
{
}

/////////////////////////////////////////////////
GearTaskPlugin::~GearTaskPlugin()
{
}

/////////////////////////////////////////////////
void GearTaskPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "GearTaskPlugin world pointer is NULL");
  GZ_ASSERT(_sdf, "GearTaskPlugin sdf pointer is NULL");
  this->world = _world;
  this->sdf = _sdf;

  // Read the SDF parameters
  sdf::ElementPtr conveyorNameElem = _sdf->GetElement("conveyor_belt_name");
  std::string conveyorName = conveyorNameElem->Get<std::string>();
  this->conveyorModel = this->world->GetModel(conveyorName);
  if (!this->conveyorModel)
  {
    gzerr << "Conveyor belt model [" << conveyorName << "] not found!\n";
  }

  sdf::ElementPtr sequence = _sdf->GetElement("object_sequence");

  sdf::ElementPtr objectElem = sequence->GetElement("object");
  while (objectElem)
  {
    // Parse the time.
    sdf::ElementPtr timeElement = objectElem->GetElement("time");
    double time = timeElement->Get<double>();

    // Parse the object type.
    sdf::ElementPtr typeElement = objectElem->GetElement("type");
    std::string type = typeElement->Get<std::string>();

    // Parse the object pose.
    sdf::ElementPtr poseElement = objectElem->GetElement("pose");
    math::Pose pose = poseElement->Get<math::Pose>();

    // Add the object to the collection.
    Object obj = {time, type, pose};
    this->objects.push_back(obj);

    objectElem = objectElem->GetNextElement("object");
  }

  std::sort(this->objects.begin(), this->objects.end());

  this->connection = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&GearTaskPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void GearTaskPlugin::Init()
{

}

/////////////////////////////////////////////////
void GearTaskPlugin::Reset()
{

}

/////////////////////////////////////////////////
void GearTaskPlugin::OnUpdate()
{
  if (this->objects.empty())
    return;

  // Check whether spawn a new object from the list.
  if (this->world->GetSimTime() >= this->objects.front().time)
  {
    auto obj = this->objects.front();
    gzdbg << "Object spawned: " << obj << std::endl;

    std::ostringstream newModelStr;

    newModelStr <<
      "<sdf version='" << SDF_VERSION << "'>\n"
      "  <include>\n"
      "    <pose>" << obj.pose << "</pose>\n"
      "    <uri>model://" << obj.type << "</uri>\n"
      "  </include>\n"
      "</sdf>\n";

    this->world->InsertModelString(newModelStr.str());
    this->objects.erase(this->objects.begin());
  }
}
