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
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include <string>

// ROS
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/ConveyorBeltState.h>
#include <osrf_gear/ProximitySensorState.h>
#include <ros/ros.h>

namespace gazebo
{
class ROSConveyorController : public WorldPlugin
{
  private: ros::NodeHandle* rosnode;
  private: ros::ServiceClient controlClient;
  private: ros::Subscriber sensorSub;
  private: physics::WorldPtr world;
  private: double beltVelocity;

  public: ~ROSConveyorController()
  {
    this->rosnode->shutdown();
  }

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->world = _parent;
    this->rosnode = new ros::NodeHandle("");

    // Create a subscriber for the proximity sensor output 
    std::string sensorStateChangeTopic = "sensor_output_change";
    this->sensorSub = 
      this->rosnode->subscribe(sensorStateChangeTopic, 1000,
        &ROSConveyorController::OnSensorStateChange, this);

    // Create a client for the conveyor control commands 
    std::string conveyorControlTopic = "conveyor_control";
    this->controlClient = this->rosnode->serviceClient<osrf_gear::ConveyorBeltControl>(
      conveyorControlTopic);

    this->beltVelocity = 0.5;

    // Turn belt on
    osrf_gear::ConveyorBeltState controlMsg;
    controlMsg.velocity = this->beltVelocity;
    osrf_gear::ConveyorBeltControl controlRequest;
    controlRequest.request.state = controlMsg;
    this->controlClient.call(controlRequest);
  }

  private: void OnSensorStateChange(const osrf_gear::ProximitySensorState::ConstPtr &_msg)
  {
    gzdbg << "Sensor state changed\n";

    bool sensorValue = _msg->state;
    bool controlCommand; // on (true) or off (false)
    if (_msg->normally_open) {
      controlCommand = !sensorValue;
    }
    else {
      controlCommand = sensorValue;
    }

    osrf_gear::ConveyorBeltState controlMsg;
    controlMsg.velocity = this->beltVelocity * controlCommand;
    osrf_gear::ConveyorBeltControl controlRequest;
    controlRequest.request.state = controlMsg;
    this->controlClient.call(controlRequest);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ROSConveyorController)
}
