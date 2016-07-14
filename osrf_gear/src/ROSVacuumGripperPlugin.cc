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

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include "osrf_gear/ROSVacuumGripperPlugin.hh"

namespace gazebo
{
  /// \internal
  /// \brief Private data for the ROSVacuumGripperPlugin class.
  struct ROSVacuumGripperPluginPrivate
  {
    /// \brief for setting ROS name space.
    public: std::string robotNamespace;

    /// \brief ROS node handle.
    public: std::unique_ptr<ros::NodeHandle> rosnode;

    /// \brief Subscribes to a topic that controls the suction of the gripper.
    public: ros::Subscriber gripperSub;

    /// \brief Custom callback queue.
    public: ros::CallbackQueue queue;

    // \brief Custom callback queue thread.
    public: boost::thread callbackQueueThread;
  };
}

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSVacuumGripperPlugin);

/////////////////////////////////////////////////
ROSVacuumGripperPlugin::ROSVacuumGripperPlugin()
  : VacuumGripperPlugin(),
    dataPtr(new ROSVacuumGripperPluginPrivate)
{
}

/////////////////////////////////////////////////
ROSVacuumGripperPlugin::~ROSVacuumGripperPlugin()
{
  this->dataPtr->queue.clear();
  this->dataPtr->queue.disable();
  this->dataPtr->rosnode->shutdown();
  this->dataPtr->callbackQueueThread.join();
}

/////////////////////////////////////////////////
void ROSVacuumGripperPlugin::Load(physics::ModelPtr _parent,
    sdf::ElementPtr _sdf)
{
  // load parameters
  this->dataPtr->robotNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->dataPtr->robotNamespace = _sdf->GetElement(
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

  std::string topic = "vacuum_gripper";
  if (_sdf->HasElement("control_topic"))
    topic = _sdf->Get<std::string>("control_topic");

  VacuumGripperPlugin::Load(_parent, _sdf);

  this->dataPtr->rosnode.reset(
    new ros::NodeHandle(this->dataPtr->robotNamespace));

  ros::SubscribeOptions so =
    ros::SubscribeOptions::create<std_msgs::Bool>(topic, 1,
        boost::bind(&ROSVacuumGripperPlugin::OnGripperControl, this, _1),
        ros::VoidPtr(), &this->dataPtr->queue);

  this->dataPtr->gripperSub = this->dataPtr->rosnode->subscribe(so);

  // start custom queue for elevator
  this->dataPtr->callbackQueueThread =
    boost::thread(boost::bind(&ROSVacuumGripperPlugin::QueueThread, this));
}

/////////////////////////////////////////////////
void ROSVacuumGripperPlugin::Reset()
{
  VacuumGripperPlugin::Reset();
}

/////////////////////////////////////////////////
void ROSVacuumGripperPlugin::OnGripperControl(
    const std_msgs::Bool::ConstPtr &_msg)
{
  if (_msg->data)
    this->Enable();
  else
    this->Disable();
}

/////////////////////////////////////////////////
void ROSVacuumGripperPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (this->dataPtr->rosnode->ok())
    this->dataPtr->queue.callAvailable(ros::WallDuration(timeout));
}
