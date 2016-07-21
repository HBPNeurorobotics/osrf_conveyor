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

  /// \brief A plugin that orchestrates an ARIAC task. First of all, it loads a
  /// description of the goals. Here's an example:
  ///
  ///  <goal>
  ///    <time>5.0</time>
  ///    <!-- 1st kit -->
  ///    <kit>
  ///      <object>
  ///        <type>coke_can</type>
  ///        <pose>-1 2.5 0.2 0 0 0</pose>
  ///      </object>
  ///      <object>
  ///        <type>cordless_drill/</type>
  ///        <pose>-1 2.5 0.2 0 0 0</pose>
  ///      </object>
  ///      <object>
  ///        <type>beer</type>
  ///        <pose>-1 2.5 0.2 0 0 0</pose>
  ///      </object>
  ///    </kit>
  ///    <!-- 2nd kit -->
  ///    <kit>
  ///      <object>
  ///        <type>coke_can</type>
  ///        <pose>-1 2.5 0.2 0 0 0</pose>
  ///      </object>
  ///      <object>
  ///        <type>cordless_drill/</type>
  ///        <pose>-1 2.5 0.2 0 0 0</pose>
  ///      </object>
  ///      <object>
  ///        <type>beer</type>
  ///        <pose>-1 2.5 0.2 0 0 0</pose>
  ///      </object>
  ///    </kit>
  ///  </goal>
  ///
  /// A task can have multiple goals. Each goal has a time element. At that
  /// point in simulation time, the goal will be notified to the team.
  /// A goal is composed by a positive number of kits. A kit is composed by
  /// a positive number of objects. An object contains a type (e.g.: coke_can)
  /// and a pose. The pose is the target pose where the object should be placed
  /// on a destination tray.
  ///
  /// After loading the goals, the plugin will use a simple finite state machine
  /// to handle the different tasks to do.
  ///
  /// The initial state is called "init" and there's not much to do when the
  /// plugin is in this state. The state of the Gazebo task manager is
  /// periodically published on a ROS topic "gazebo_task/state". This topic can
  /// be changed using the SDF element <gazebo_task_state_topic>. The plugin is
  /// waiting for a ROS message on topic "team_task/state". This topic shows
  /// the state of the team performing the task. When the value of a message
  /// received is "ready", the plugin will transition its internal state towards
  /// "ready" too.
  ///
  /// The "ready" state is considered simulation time zero for notifying the
  /// goals to the teams. A <time>1.0</time> inside a <goal> element will be
  /// notified 1 second after entering in the "ready" state. The goal will be
  /// published using a ROS topic on the topic ${robot_namespace}/goal or
  /// ${robot_namespace}/${goals_topic} if the parameter <goals_topic> is passed
  /// in to the plugin. Also, when the plugin is in this state, it will use the
  /// conveyor activation topic to communicate with the Population plugin and
  /// enable the population of the conveyor belt. The element
  /// <conveyor_activation_topic> should match the topic used in the
  /// <activation_topic> of the Population plugin. After enabling the conveyor
  /// belt, the state changes to "go".
  ///
  /// In "go" state, the plugin will be updating the function that processes
  /// the goals. This is essentially checking if it's time to announce a new
  /// goal.
  /// ToDo: Check if the team has completed all the goals or the maximum time
  /// limit has been reached. In that case, move to "finish" state.
  ///
  /// In "finish" state there's nothing to do.
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

    /// \brief Start populating the conveyor belt.
    protected: void PopulateConveyorBelt();

    /// \brief Callback received when there's an status update.
    public: void StatusCallback(const std_msgs::String::ConstPtr &_msg);

    /// \brief Private data pointer.
    private: std::unique_ptr<ROSAriacTaskManagerPluginPrivate> dataPtr;
  };
}
#endif
