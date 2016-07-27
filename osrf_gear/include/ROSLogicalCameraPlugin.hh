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

#ifndef _ROS_LOGICAL_CAMERA_PLUGIN_HH_
#define _ROS_LOGICAL_CAMERA_PLUGIN_HH_

#include <sdf/sdf.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/msgs/logical_camera_image.pb.h"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"
#include "gazebo/transport/TransportTypes.hh"

// ROS
#include <ros/ros.h>

namespace gazebo
{
  /// \brief ROS publisher for the logical camera
  class ROSLogicalCameraPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ROSLogicalCameraPlugin();

    /// \brief Destructor
    public: virtual ~ROSLogicalCameraPlugin();

    /// \brief Model that contains the logical camera
    protected: physics::ModelPtr model;

    /// \brief Link that holds the logical camera
    protected: physics::LinkPtr cameraLink;

    /// \brief The logical camera sensor
    protected: sensors::SensorPtr sensor;

    /// \brief Load the plugin.
    /// \param[in] _parent Pointer to the parent model
    /// \param[in] _sdf Pointer to the SDF element of the plugin.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Searches the model links for a logical camera sensor
    protected: void FindLogicalCamera();

    /// \brief Callback for when logical camera images are received
    /// \param[in] _msg The logical camera image
    public: void OnImage(ConstLogicalCameraImagePtr &_msg);

    /// \brief Node for communication with gazebo
    protected: transport::NodePtr node;

    /// \brief Subscription to logical camera image messages from gazebo
    protected: transport::SubscriberPtr imageSub;

    /// \brief for setting ROS name space
    protected: std::string robotNamespace;

    /// \brief ros node handle
    protected: ros::NodeHandle *rosnode;

    /// \brief ROS publisher for the logical camera image
    protected: ros::Publisher imagePub;
  };
}
#endif
