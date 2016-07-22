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
#include "osrf_gear/LogicalCameraImage.h"
#include "ROSLogicalCameraPlugin.hh"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <string>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSLogicalCameraPlugin);

/////////////////////////////////////////////////
ROSLogicalCameraPlugin::ROSLogicalCameraPlugin()
{
}

/////////////////////////////////////////////////
ROSLogicalCameraPlugin::~ROSLogicalCameraPlugin()
{
  this->rosnode->shutdown();
}

/////////////////////////////////////////////////
void ROSLogicalCameraPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // load parameters
  this->robotNamespace = "logical_camera";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace = _sdf->GetElement(
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

  this->model = _parent;
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  std::string imageTopic_gazebo = "~/logical_camera/link/logical_camera/models";
  if (_sdf->HasElement("image_topic_gazebo"))
    imageTopic_gazebo = _sdf->Get<std::string>("image_topic_gazebo");

  std::string imageTopic_ros = "image";
  if (_sdf->HasElement("image_topic_ros"))
    imageTopic_gazebo = _sdf->Get<std::string>("image_topic_ros");

  this->rosnode = new ros::NodeHandle(this->robotNamespace);

  this->imageSub = this->node->Subscribe(imageTopic_gazebo,
          &ROSLogicalCameraPlugin::OnImage, this);

  gzdbg << "Image loaded\n";
  this->imagePub = this->rosnode->advertise<osrf_gear::LogicalCameraImage>(imageTopic_ros, 1, true);
}

/////////////////////////////////////////////////
void ROSLogicalCameraPlugin::OnImage(ConstLogicalCameraImagePtr &_msg)
{
  osrf_gear::LogicalCameraImage imageMsg;
  for (int i = 0; i < _msg->model_size(); ++i)
  {
    osrf_gear::Model model;
    model.name = _msg->model(i).name();
    imageMsg.models.push_back(model);
  }
  this->imagePub.publish(imageMsg);
}
