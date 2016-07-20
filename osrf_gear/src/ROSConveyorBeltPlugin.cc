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
#include "ROSConveyorBeltPlugin.hh"

#include <string>

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(ROSConveyorBeltPlugin);

/////////////////////////////////////////////////
ROSConveyorBeltPlugin::ROSConveyorBeltPlugin()
{
}

/////////////////////////////////////////////////
ROSConveyorBeltPlugin::~ROSConveyorBeltPlugin()
{
  this->rosnode_->shutdown();
}

/////////////////////////////////////////////////
void ROSConveyorBeltPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load parameters
  this->robotNamespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace_ = _sdf->GetElement(
        "robotNamespace")->Get<std::string>() + "/";
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  std::string topic = "conveyor_control";
  if (_sdf->HasElement("topic"))
    topic = _sdf->Get<std::string>("topic");

  ConveyorBeltPlugin::Load(_parent, _sdf);

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace_);

  this->controlCommandSub_ = this->rosnode_->subscribe(topic, 1000,
    &ROSConveyorBeltPlugin::OnControlCommand, this);
}

/////////////////////////////////////////////////
void ROSConveyorBeltPlugin::OnControlCommand(const osrf_gear::ConveyorBeltState::ConstPtr &_msg)
{
  gzdbg << "Control command received\n";
  this->SetVelocity(_msg->velocity);
}
