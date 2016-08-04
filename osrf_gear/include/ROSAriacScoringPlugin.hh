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
/*
 * Desc: ARIAC scorer.
 * Author: Deanna Hood
 */
#ifndef _ROS_ARIAC_SCORER_HH_
#define _ROS_ARIAC_SCORER_HH_

#include <string>

#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/util/system.hh>
#include <osrf_gear/ARIAC.hh>
#include <osrf_gear/Goal.h>
#include "ROSAriacKitTray.hh"

namespace gazebo
{
  /// \brief A plugin for a scoring the state of the ARIAC game.
  class GAZEBO_VISIBLE ROSAriacScoringPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: ROSAriacScoringPlugin();

    /// \brief Destructor.
    public: virtual ~ROSAriacScoringPlugin();

    /// \brief Load the world plugin.
    /// \param[in] _model Pointer to the world that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the world update event.
    protected: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Process a new goal.
    protected: void ProcessNewGoal();

    /// \brief Calculate the score for the trays given the objects in them
    /// \return The score for the goal.
    protected: ariac::GoalScore ScoreTrays();

    /// \brief Helper function for filling a Kit from a kit ROS message.
    public: static void FillKitFromMsg(const osrf_gear::Kit &kitMsg, ariac::Kit &kit);

    /// \brief Pointer to the gazebo world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to the update event connection.
    protected: event::ConnectionPtr updateConnection;

    /// \brief ROS node handle.
    protected: ros::NodeHandle *rosNode;

    /// \brief Subscriber for goal messages.
    protected: ros::Subscriber goalSub;

    /// \brief Subscriber for tray state messages.
    protected: ros::Subscriber traySub;

    /// \brief Callback for receiving goal message.
    public: void OnGoalReceived(const osrf_gear::Goal::ConstPtr & goalMsg);

    /// \brief Callback for receiving tray state message.
    public: void OnTrayInfo(const osrf_gear::Kit::ConstPtr & trayMsg);

    /// \brief The trays to monitor the score of.
    protected: std::map<std::string, ariac::KitTray> kitTrays;

    /// \brief Mutex for protecting the kit trays being monitored.
    protected: mutable boost::mutex kitTraysMutex;

    /// \brief Flag for signalling new tray info to process.
    protected: bool newTrayInfoReceived;

    /// \brief Flag for signalling new goal to process.
    protected: bool newGoalReceived;

    /// \brief Goal receivd from goal messages.
    protected: ariac::Goal newGoal;

    /// \brief Parameters to use for calculating scores.
    protected: ariac::ScoringParameters scoringParameters;

    /// \brief The score of the current goal.
    protected: ariac::GoalScore goalScore;

    /// \brief The score of the run.
    protected: ariac::GameScore gameScore;

  };
}
#endif

