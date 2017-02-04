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
#include "ROSAGVPlugin.hh"

#include <gazebo/common/common.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Time.hh>
#include <ignition/math.hh>
#include <osrf_gear/SubmitTray.h>
#include <std_srvs/Trigger.h>

#include <string>

namespace gazebo
{
  /// \internal
  /// \brief Private data for the ROSAGVPlugin class.
  struct ROSAGVPluginPrivate
  {
    /// \brief Name of the AGV
    public: std::string agvName;

    /// \brief World pointer
    public: physics::WorldPtr world;

    /// \brief Pointer to the update event connection
    public: event::ConnectionPtr updateConnection;

    /// \brief for setting ROS name space
    public: std::string robotNamespace;

    /// \brief ros node handle
    public: ros::NodeHandle *rosnode;

    /// \brief Receives service calls for controlling the AGV
    public: ros::ServiceServer rosService;

    /// \brief Client for submitting trays for inspection
    public: ros::ServiceClient rosSubmitTrayClient;

    /// \brief Client for clearing this AGV's tray
    public: ros::ServiceClient rosClearTrayClient;

    /// \brief Robot animation for delivering the tray
    public: gazebo::common::PoseAnimationPtr deliverTrayAnimation;

    /// \brief Robot animation for the AGV returning to its base
    public: gazebo::common::PoseAnimationPtr returnAnimation;

    /// \brief Pointer to the model
    public: gazebo::physics::ModelPtr model;

    /// \brief Type of kit assigned to the AGV
    public: std::string kitType;

    /// \brief The state of the AGV
    public: std::string currentState;

    /// \brief The time the last tray was delivered
    public: common::Time deliveryTime;
  };
}

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSAGVPlugin);

/////////////////////////////////////////////////
ROSAGVPlugin::ROSAGVPlugin()
  : dataPtr(new ROSAGVPluginPrivate)
{
}

/////////////////////////////////////////////////
ROSAGVPlugin::~ROSAGVPlugin()
{
  this->dataPtr->rosnode->shutdown();
}

/////////////////////////////////////////////////
void ROSAGVPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  std::string index;

  if (_sdf->HasElement("index"))
  {
    index = _sdf->Get<std::string>("index");
  }
  else
  {
    gzerr << "AGV is missing an index. The AGV will not work.\n";
  }

  this->dataPtr->world = _parent->GetWorld();

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

  this->dataPtr->agvName = std::string("agv") + index;

  std::string agvControlTopic = "/ariac/" + this->dataPtr->agvName;
  ROS_DEBUG_STREAM("Using AGV control service topic: " << agvControlTopic);

  std::string submitTrayTopic = "submit_tray";
  if (_sdf->HasElement("submit_tray_service_name"))
    submitTrayTopic = _sdf->Get<std::string>("submit_tray_service_name");
  ROS_DEBUG_STREAM("Using submit tray service topic: " << submitTrayTopic);

  std::string clearTrayServiceName = "clear_tray";
  if (_sdf->HasElement("clear_tray_service_name"))
    clearTrayServiceName = _sdf->Get<std::string>("clear_tray_service_name");
  ROS_DEBUG_STREAM("Using clear tray service topic: " << clearTrayServiceName);

  this->dataPtr->rosnode = new ros::NodeHandle(this->dataPtr->robotNamespace);

  this->dataPtr->deliverTrayAnimation.reset(new gazebo::common::PoseAnimation(this->dataPtr->agvName, 10, false));

  gazebo::common::PoseKeyFrame *key = this->dataPtr->deliverTrayAnimation->CreateKeyFrame(0);
  key->Translation(ignition::math::Vector3d(0.3, 3.3, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 3.1415));

  key = this->dataPtr->deliverTrayAnimation->CreateKeyFrame(4);
  key->Translation(ignition::math::Vector3d(-4.2, 3.8, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 3.1415));

  key = this->dataPtr->deliverTrayAnimation->CreateKeyFrame(10);
  key->Translation(ignition::math::Vector3d(-4.2, 9.45, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 3.1415));

  this->dataPtr->returnAnimation.reset(new gazebo::common::PoseAnimation(this->dataPtr->agvName, 12, false));

  key = this->dataPtr->returnAnimation->CreateKeyFrame(0);
  key->Translation(ignition::math::Vector3d(-4.2, 9.45, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 3.1415));

  key = this->dataPtr->returnAnimation->CreateKeyFrame(6);
  key->Translation(ignition::math::Vector3d(-4.2, 3.8, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 3.1415));

  key = this->dataPtr->returnAnimation->CreateKeyFrame(12);
  key->Translation(ignition::math::Vector3d(0.3, 3.3, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 3.1415));

  this->dataPtr->model = _parent;

  this->dataPtr->rosService = this->dataPtr->rosnode->advertiseService(agvControlTopic,
      &ROSAGVPlugin::OnCommand, this);

  // Client for submitting trays for inspection.
  this->dataPtr->rosSubmitTrayClient =
    this->dataPtr->rosnode->serviceClient<osrf_gear::SubmitTray>(submitTrayTopic);

  // Client for clearing trays.
  this->dataPtr->rosClearTrayClient =
    this->dataPtr->rosnode->serviceClient<std_srvs::Trigger>(clearTrayServiceName);

  this->dataPtr->currentState = "ready_to_deliver";

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&ROSAGVPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void ROSAGVPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  auto currentSimTime = this->dataPtr->world->GetSimTime();
  if (this->dataPtr->currentState == "delivering")
  {
    if (this->dataPtr->deliverTrayAnimation->GetTime() >= this->dataPtr->deliverTrayAnimation->GetLength())
    {
      gzdbg << "Delivery animation finished." << std::endl;
      this->dataPtr->deliveryTime = currentSimTime;
      this->dataPtr->currentState = "delivered";
    }
  }
  if (this->dataPtr->currentState == "delivered")
  {
    if ((currentSimTime - this->dataPtr->deliveryTime) < 2.0)
      return;

    // Make a service call to submit the tray for inspection.
    if (!this->dataPtr->rosSubmitTrayClient.exists())
    {
      this->dataPtr->rosSubmitTrayClient.waitForExistence();
    }
    osrf_gear::SubmitTray submit_srv;
    submit_srv.request.tray_id = this->dataPtr->agvName + "::kit_tray::kit_tray::tray";
    submit_srv.request.kit_type = this->dataPtr->kitType;
    this->dataPtr->rosSubmitTrayClient.call(submit_srv);
    if (!submit_srv.response.success)
    {
      ROS_ERROR_STREAM("Failed to submit tray for inspection.");
    }
    else
    {
      ROS_INFO_STREAM("Result of inspection: " << submit_srv.response.inspection_result);
    }

    // Make a service call to clear the tray.
    if (!this->dataPtr->rosClearTrayClient.exists())
    {
      this->dataPtr->rosClearTrayClient.waitForExistence();
    }
    std_srvs::Trigger clear_srv;
    this->dataPtr->rosClearTrayClient.call(clear_srv);
    if (!clear_srv.response.success)
    {
      ROS_ERROR_STREAM("Failed to clear tray.");
    }
    else
    {
      ROS_INFO_STREAM("Tray successfully cleared.");
    }

    // Trigger the return animation.
    this->dataPtr->returnAnimation->SetTime(0);
    this->dataPtr->model->SetAnimation(this->dataPtr->returnAnimation);
    this->dataPtr->currentState = "returning";
  }
  if (this->dataPtr->currentState == "returning")
  {
    if (this->dataPtr->returnAnimation->GetTime() >= this->dataPtr->returnAnimation->GetLength())
    {
      gzdbg << "Return animation finished." << std::endl;
      this->dataPtr->currentState = "ready_to_deliver";
    }
  }

}

/////////////////////////////////////////////////
bool ROSAGVPlugin::OnCommand(
  osrf_gear::AGVControl::Request &_req,
  osrf_gear::AGVControl::Response &_res)
{
  if (this->dataPtr->currentState != "ready_to_deliver")
  {
    ROS_ERROR_STREAM("AGV not successfully triggered as it was not ready to deliver trays.");
    _res.success = false;
    return true;
  }

  // Trigger the tray delivery animation
  this->dataPtr->deliverTrayAnimation->SetTime(0);
  this->dataPtr->model->SetAnimation(this->dataPtr->deliverTrayAnimation);
  this->dataPtr->kitType = _req.kit_type;
  ROS_INFO_STREAM("AGV successfully triggered.");
  _res.success = true;
  this->dataPtr->currentState = "delivering";

  return true;
}
