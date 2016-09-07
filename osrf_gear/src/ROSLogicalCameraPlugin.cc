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

#include "ROSLogicalCameraPlugin.hh"

#include "osrf_gear/ARIAC.hh"
#include "osrf_gear/LogicalCameraImage.h"

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>

#include <algorithm>
#include <sstream>
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

  this->onlyPublishKnownModels = false;
  if (_sdf->HasElement("known_model_types"))
  {
    this->onlyPublishKnownModels = true;
    this->knownModelTypes.clear();
    sdf::ElementPtr knownModelTypesElem = _sdf->GetElement("known_model_types");
    if (!knownModelTypesElem->HasElement("type"))
    {
      gzerr << "Unable to find <type> elements in the <known_model_types> section\n";
      return;
    }
    sdf::ElementPtr knownModelTypeElem = knownModelTypesElem->GetElement("type");
    while (knownModelTypeElem)
    {
      // Parse the model type, which is encoded in model names.
      std::string type = knownModelTypeElem->Get<std::string>();

      ROS_DEBUG_STREAM("New known model type: " << type);
      this->knownModelTypes.push_back(type);
      knownModelTypeElem = knownModelTypeElem->GetNextElement("type");
    }
  }

  this->model = _parent;
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());
  this->rosnode = new ros::NodeHandle(this->robotNamespace);

  this->FindLogicalCamera();
  if (!this->sensor)
  {
    gzerr << "No logical camera found on any link\n";
    return;
  }

  std::string imageTopic_ros = _parent->GetName();
  if (_sdf->HasElement("image_topic_ros")) {
    imageTopic_ros = _sdf->Get<std::string>("image_topic_ros");
  }

  this->imageSub = this->node->Subscribe(this->sensor->Topic(),
          &ROSLogicalCameraPlugin::OnImage, this);
  gzdbg << "Subscribing to gazebo topic: " << this->sensor->Topic() << "\n";

  this->imagePub = this->rosnode->advertise<osrf_gear::LogicalCameraImage>(imageTopic_ros, 1, true);
  gzdbg << "Publishing to ROS topic: " << imagePub.getTopic() << "\n";
}

void ROSLogicalCameraPlugin::FindLogicalCamera()
{
  sensors::SensorManager* sensorManager = sensors::SensorManager::Instance();

  // Go through each link's sensors until a logical camera is found
  for (physics::LinkPtr link : this->model->GetLinks())
  {
    for (unsigned int i = 0; i < link->GetSensorCount(); ++i)
    {
      sensors::SensorPtr sensor = sensorManager->GetSensor(link->GetSensorName(i));
      if (sensor->Type() == "logical_camera")
      {
        this->sensor = sensor;
        break;
      }
    }
    if (this->sensor)
    {
      this->cameraLink = link;
      break;
    }
  }
}

/////////////////////////////////////////////////
void ROSLogicalCameraPlugin::OnImage(ConstLogicalCameraImagePtr &_msg)
{
  osrf_gear::LogicalCameraImage imageMsg;
  msgs::Vector3d cameraPosition = _msg->pose().position();
  msgs::Quaternion cameraOrientation = _msg->pose().orientation();
  imageMsg.pose.position.x = cameraPosition.x();
  imageMsg.pose.position.y = cameraPosition.y();
  imageMsg.pose.position.z = cameraPosition.z();
  imageMsg.pose.orientation.x = cameraOrientation.x();
  imageMsg.pose.orientation.y = cameraOrientation.y();
  imageMsg.pose.orientation.z = cameraOrientation.z();
  imageMsg.pose.orientation.w = cameraOrientation.w();

  std::ostringstream logStream;
  for (int i = 0; i < _msg->model_size(); ++i)
  {
    std::string modelType = ariac::DetermineModelType(_msg->model(i).name());

    // Check if there are restrictions on which models to publish
    if (this->onlyPublishKnownModels)
    {
      // Only publish the model if its type is one of the known types
      auto it = std::find(this->knownModelTypes.begin(), this->knownModelTypes.end(), modelType);
      bool knownModel = it != this->knownModelTypes.end();
      if (!knownModel)
      {
        logStream << "Not publishing model of type: " << modelType << std::endl;
        continue;
      }
    }
    msgs::Vector3d position = _msg->model(i).pose().position();
    msgs::Quaternion orientation = _msg->model(i).pose().orientation();
    osrf_gear::Model modelMsg;
    modelMsg.pose.position.x = position.x();
    modelMsg.pose.position.y = position.y();
    modelMsg.pose.position.z = position.z();
    modelMsg.pose.orientation.x = orientation.x();
    modelMsg.pose.orientation.y = orientation.y();
    modelMsg.pose.orientation.z = orientation.z();
    modelMsg.pose.orientation.w = orientation.w();
    modelMsg.type = modelType;
    imageMsg.models.push_back(modelMsg);
  }
  if (!logStream.str().empty())
  {
    ROS_DEBUG_THROTTLE(1, "%s", logStream.str().c_str());
  }
  this->imagePub.publish(imageMsg);
}
