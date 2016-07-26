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
#include <mutex>
#include <ostream>
#include <string>
#include <vector>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/msgs/gz_string.pb.h>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Matrix4.hh>
#include <sdf/sdf.hh>

#include "osrf_gear/PopulationPlugin.hh"

namespace gazebo
{
  /// \internal
  /// \brief Private data for the PopulationPlugin class.
  struct PopulationPluginPrivate
  {
    /// \brief World pointer.
    public: physics::WorldPtr world;

    /// \brief SDF pointer.
    public: sdf::ElementPtr sdf;

    /// \brief Class to store information about each object to be populated.
    public: class Object
            {
             /// \brief Less than operator.
             /// \param[in] _obj Other object to compare
             /// \return True if this < _obj
             public: bool operator<(const Object &_obj) const
             {
               return this->time < _obj.time;
             }

             /// \brief Stream insertion operator.
             /// \param[in] _out output stream
             /// \param[in] _obj object to output
             /// \return The output stream
             public: friend std::ostream &operator<<(std::ostream &_out,
                                                     const Object &_obj)
             {
               _out << _obj.type << std::endl;
               _out << "  Time: [" << _obj.time << "]" << std::endl;
               _out << "  Pose: [" << _obj.pose << "]" << std::endl;
               return _out;
             }

              /// \brief Simulation time in which the object should be spawned.
              public: double time;

              /// \brief Object type.
              public: std::string type;

              /// \brief Pose in which the object should be placed.
              public: math::Pose pose;
            };

    /// \brief Collection of objects to be spawned.
    public: std::vector<Object> objects;

    /// \brief Contains the entire collection of objects. This is used for
    /// inserting the objects in a cyclic way.
    public: std::vector<Object> initialObjects;

    /// \brief Connection event.
    public: event::ConnectionPtr connection;

    /// \brief The time specified in the object is relative to this time.
    public: common::Time startTime;

    /// \brief When true, "objects" will be repopulated when the object queue
    /// is empty, creating an infinite supply of objects.
    public: bool loopForever = false;

    /// \brief Link which the object poses use as their frame of reference.
    public: physics::LinkPtr link;

    /// \brief Node for communication.
    public: transport::NodePtr node;

    /// \brief Subscriber to the activation topic.
    public: transport::SubscriberPtr activationSub;

    /// \brief If true, the objects will start populating.
    public: bool enabled = false;

    /// \brief Mutex to avoid race conditions.
    public: std::mutex mutex;
  };
}

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(PopulationPlugin)

/////////////////////////////////////////////////
PopulationPlugin::PopulationPlugin()
  : dataPtr(new PopulationPluginPrivate)
{
}

/////////////////////////////////////////////////
PopulationPlugin::~PopulationPlugin()
{
}

/////////////////////////////////////////////////
void PopulationPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "PopulationPlugin world pointer is NULL");
  GZ_ASSERT(_sdf, "PopulationPlugin sdf pointer is NULL");
  this->dataPtr->world = _world;
  this->dataPtr->sdf = _sdf;

  // Read the SDF parameters
  if (_sdf->HasElement("loop_forever"))
  {
    sdf::ElementPtr loopElem = _sdf->GetElement("loop_forever");
    this->dataPtr->loopForever = loopElem->Get<bool>();
  }

  if (_sdf->HasElement("link_frame"))
  {
    std::string linkName = _sdf->Get<std::string>("link_frame");
    this->dataPtr->link =
      boost::dynamic_pointer_cast<physics::Link>(this->dataPtr->world->GetEntity(linkName));
  }

  if (!_sdf->HasElement("object_sequence"))
  {
    gzerr << "PopulationPlugin: Unable to find <object_sequence> element\n";
    return;
  }

  sdf::ElementPtr sequence = _sdf->GetElement("object_sequence");

  sdf::ElementPtr objectElem = sequence->GetElement("object");
  while (objectElem)
  {
    // Parse the time.
    if (!objectElem->HasElement("time"))
    {
      gzerr << "PopulationPlugin: Unable to find <time> in object\n";
      objectElem = objectElem->GetNextElement("object");
      continue;
    }
    sdf::ElementPtr timeElement = objectElem->GetElement("time");
    double time = timeElement->Get<double>();

    // Parse the object type.
    if (!objectElem->HasElement("type"))
    {
      gzerr << "PopulationPlugin: Unable to find <type> in object.\n";
      objectElem = objectElem->GetNextElement("object");
      continue;
    }
    sdf::ElementPtr typeElement = objectElem->GetElement("type");
    std::string type = typeElement->Get<std::string>();

    // Parse the object pose (optional).
    math::Pose pose;
    if (objectElem->HasElement("pose"))
    {
      sdf::ElementPtr poseElement = objectElem->GetElement("pose");
      pose = poseElement->Get<math::Pose>();
    }

    // Add the object to the collection.
    PopulationPluginPrivate::Object obj = {time, type, pose};
    this->dataPtr->initialObjects.push_back(obj);

    objectElem = objectElem->GetNextElement("object");
  }
  std::sort(this->dataPtr->initialObjects.begin(),
    this->dataPtr->initialObjects.end());

  // Listen on the activation topic, if present. This topic is used for
  // manual activation.
  if (_sdf->HasElement("activation_topic"))
  {
    // Create and initialize the node.
    this->dataPtr->node = transport::NodePtr(new transport::Node());
    this->dataPtr->node->Init();

    // Subscribe to the activation topic.
    this->dataPtr->activationSub = this->dataPtr->node->Subscribe(
        _sdf->Get<std::string>("activation_topic"),
        &PopulationPlugin::OnActivation, this);
  }
  else
    this->Restart();

  this->dataPtr->connection = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&PopulationPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void PopulationPlugin::Restart()
{
  this->dataPtr->enabled = true;
  this->dataPtr->startTime = this->dataPtr->world->GetSimTime();
  this->dataPtr->objects = this->dataPtr->initialObjects;
}

/////////////////////////////////////////////////
void PopulationPlugin::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!this->dataPtr->enabled)
    return;

  if (this->dataPtr->objects.empty())
  {
    if (this->dataPtr->loopForever)
      this->Restart();
    else
      return;
  }

  // Check whether spawn a new object from the list.
  auto elapsed = this->dataPtr->world->GetSimTime() - this->dataPtr->startTime;
  if (elapsed.Double() >= this->dataPtr->objects.front().time)
  {
    auto obj = this->dataPtr->objects.front();
    if (this->dataPtr->link)
    {
      auto linkPose = this->dataPtr->link->GetWorldPose().Ign();
      ignition::math::Matrix4d transMat(linkPose);
      ignition::math::Matrix4d pose_local(obj.pose.Ign());
      obj.pose = (transMat * pose_local).Pose();
    }
    gzdbg << "Object spawned: " << obj << std::endl;

    std::ostringstream newModelStr;

    newModelStr <<
      "<sdf version='" << SDF_VERSION << "'>\n"
      "  <include>\n"
      "    <pose>" << obj.pose << "</pose>\n"
      "    <uri>model://" << obj.type << "</uri>\n"
      "  </include>\n"
      "</sdf>\n";

    this->dataPtr->world->InsertModelString(newModelStr.str());
    this->dataPtr->objects.erase(this->dataPtr->objects.begin());
  }
}

/////////////////////////////////////////////////
void PopulationPlugin::OnActivation(ConstGzStringPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (_msg->data() == "restart")
    this->Restart();
  else if (_msg->data() == "stop")
    this->dataPtr->enabled = false;
  else
    gzerr << "Unknown activation command [" << _msg->data() << "]" << std::endl;
}
