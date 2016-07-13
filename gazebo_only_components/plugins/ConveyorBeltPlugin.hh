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
/*
 * Desc: Conveyor Belt Plugin
 * Author: Nate Koenig mod by John Hsu and Deanna Hood
 */
#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  /// \brief A plugin for a contact sensor. Inherit from this class to make
  /// your own contact plugin.
  class GAZEBO_VISIBLE ConveyorBeltPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: ConveyorBeltPlugin();

    /// \brief Destructor.
    public: virtual ~ConveyorBeltPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that recieves the contact sensor's update signal.
    /// Override this this function to get callbacks when the contact sensor
    /// is updated with new data.
    private: virtual void OnUpdate();

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;

    /// \brief Pointer to the world
    private: physics::WorldPtr world;

    /// \brief Pointer to this node for publishing/subscribing
    private: transport::NodePtr node;

    /// \brief Subscriber for the control commands
    private: transport::SubscriberPtr controlCommandSub;

    /// \brief Callback for responding to control commands
    private: void OnControlCommand(ConstHeaderPtr& _msg);

    /// \brief Belt speed (m/s)
    private: double beltSpeed;

    /// \brief Belt state (true: on, false: off)
    private: bool state;

    /// \brief Mutex to protect the belt state
    private: std::mutex stateMutex;

    /// \brief Generate a scoped topic name from a local one
    /// \param local local topic name
    private: std::string Topic(std::string topicName) const;

    /// \brief Name of the collision of the belt
    private: std::string beltCollisionName;

    /// \brief Pointer to the belt link
    private: physics::LinkPtr beltLink;

    /// \brief Set of pointers to links which have collisions with the belt
    private: std::set<physics::LinkPtr> contactingLinkPtrs;

    /// \brief Determine which links are ontop of the belt
    private: void CalculateContactingLinks();

    /// \brief Act on links that are ontop of the belt
    private: void ActOnContactingLinks(double speed);
  };
}
#endif

