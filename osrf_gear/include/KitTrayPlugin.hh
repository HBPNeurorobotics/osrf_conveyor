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
 * Desc: Kit tray plugin
 * Author: Deanna Hood
 */
#ifndef _GAZEBO_KIT_TRAY_PLUGIN_HH_
#define _GAZEBO_KIT_TRAY_PLUGIN_HH_

#include <string>

#include <ros/ros.h>

#include <ARIAC.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/util/system.hh>
#include <osrf_gear/Goal.h>
#include "TopContactPlugin.hh"

namespace gazebo
{
  /// \brief A plugin for a contact sensor on a kit tray.
  class GAZEBO_VISIBLE KitTrayPlugin : public TopContactPlugin
  {
    /// \brief Constructor.
    public: KitTrayPlugin();

    /// \brief Destructor.
    public: virtual ~KitTrayPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that recieves the contact sensor's update signal.
    protected: void OnUpdate();

    /// \brief Act on models that are ontop of the belt
    protected: void ActOnContactingModels();

    /// \brief Kit to be built on this tray
    protected: ariac::Kit kit;
    protected: ros::NodeHandle *rosNode;
    protected: ros::Subscriber goalSub;
    public: void OnGoalReceived(const osrf_gear::Goal::ConstPtr & goalMsg);
  };
}
#endif

