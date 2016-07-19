/*
 * Copyright 2016 Open Source Robotics Foundation
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

#ifndef _ROS_CONVEYOR_BELT_PLUGIN_HH_
#define _ROS_CONVEYOR_BELT_PLUGIN_HH_

#include <sdf/sdf.hh>

// Gazebo
#include "ConveyorBeltPlugin.hh"

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

namespace gazebo
{
  /// \brief ROS implementation of the ConveyorBeltPlugin plugin
  class ROSConveyorBeltPlugin : public ConveyorBeltPlugin
  {
    /// \brief Constructor
    public: ROSConveyorBeltPlugin();

    /// \brief Destructor
    public: virtual ~ROSConveyorBeltPlugin();

    /// \brief Load the plugin.
    /// \param[in] _parent Pointer to the parent sensor
    /// \param[in] _sdf Pointer to the SDF element of the plugin.
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Receives messages on the conveyor belt's topic.
    /// \param[in] _msg The string message that contains a command.
    public: void OnControlCommand(const std_msgs::String::ConstPtr &_msg);

    /// \brief Queu to handle callbacks.
    private: void QueueThread();

    /// \brief for setting ROS name space
    private: std::string robotNamespace_;

    /// \brief ros node handle
    private: ros::NodeHandle *rosnode_;

    /// \brief Subscribes to a topic that controls the elevator.
    private: ros::Subscriber controlCommandSub_;

    /// \brief Custom Callback Queue
    private: ros::CallbackQueue queue_;

    // \brief Custom Callback Queue thread
    private: boost::thread callbackQueueThread_;
  };
}
#endif
