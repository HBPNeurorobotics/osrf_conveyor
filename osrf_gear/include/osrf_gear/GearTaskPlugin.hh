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

#ifndef GAZEBO_GEARTASK_PLUGIN_HH_
#define GAZEBO_GEARTASK_PLUGIN_HH_

#include <ostream>
#include <vector>
#include <gazebo/common/Plugin.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE GearTaskPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: GearTaskPlugin();

    /// \brief Destructor.
    public: ~GearTaskPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Initialize the plugin.
    public: virtual void Init();

    /// \brief Reset the plugin.
    public: virtual void Reset();

    /// \brief Update the plugin.
    private: void OnUpdate();

    /// \brief World pointer.
    protected: physics::WorldPtr world;

    /// \brief SDF pointer.
    protected: sdf::ElementPtr sdf;

    /// \brief Conveyor belt model pointer.
    protected: physics::ModelPtr conveyorModel;

    /// \brief Class to store info about each object.
    protected: class Object
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

                 /// \brief Simulation in which the object should be spawned.
                 public: double time;

                 /// \brief Object type.
                 public: std::string type;

                 /// \brief Offset from the conveyor origin in which the object
                 /// should be placed.
                 public: math::Pose pose;
               };

    /// \brief Collection of objects to be spawned.
    protected: std::vector<Object> objects;

    /// \brief Connection event.
    public: event::ConnectionPtr connection;
  };
}
#endif
