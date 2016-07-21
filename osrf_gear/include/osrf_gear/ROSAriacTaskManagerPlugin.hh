/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#ifndef GAZEBO_ROS_ARIAC_TASK_MANAGER_PLUGIN_HH_
#define GAZEBO_ROS_ARIAC_TASK_MANAGER_PLUGIN_HH_

#include <memory>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <sdf/sdf.hh>
#include <std_msgs/String.h>

namespace gazebo
{
  // Forward declare private data class
  class ROSAriacTaskManagerPluginPrivate;

  /// \brief A plugin that...
  class GAZEBO_VISIBLE ROSAriacTaskManagerPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: ROSAriacTaskManagerPlugin();

    /// \brief Destructor.
    public: virtual ~ROSAriacTaskManagerPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Update the plugin.
    protected: void OnUpdate();

    /// \brief Decide whether to announce a new goal.
    protected: void ProcessGoals();

    /// \brief Callback received when there's an status update.
    public: void StatusCallback(const std_msgs::String::ConstPtr &_msg);

    /// \brief Private data pointer.
    private: std::unique_ptr<ROSAriacTaskManagerPluginPrivate> dataPtr;
  };
}
#endif
