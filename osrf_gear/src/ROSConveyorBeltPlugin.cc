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
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callbackQueueThread_.join();

  delete this->rosnode_;
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

  ros::SubscribeOptions so =
    ros::SubscribeOptions::create<std_msgs::String>(topic, 1,
        boost::bind(&ROSConveyorBeltPlugin::OnControlCommand, this, _1),
        ros::VoidPtr(), &this->queue_);

  this->controlCommandSub_ = this->rosnode_->subscribe(so);

  // start custom queue
  this->callbackQueueThread_ =
    boost::thread(boost::bind(&ROSConveyorBeltPlugin::QueueThread, this));
}

/////////////////////////////////////////////////
void ROSConveyorBeltPlugin::OnControlCommand(const std_msgs::String::ConstPtr &_msg)
{
  gzdbg << "Control command received\n";
  this->SetState(std::stoi(_msg->data));
}

/////////////////////////////////////////////////
void ROSConveyorBeltPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
    this->queue_.callAvailable(ros::WallDuration(timeout));
}
