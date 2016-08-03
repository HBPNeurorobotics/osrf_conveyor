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
 * Desc: ARIAC scoring plugin
 * Author: Deanna Hood
 */
#ifndef _ROS_ARIAC_SCORING_PLUGIN_HH_
#define _ROS_ARIAC_SCORING_PLUGIN_HH_

#include <string>

#include <ros/ros.h>

#include <ARIAC.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/util/system.hh>
#include <osrf_gear/Goal.h>

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

    /// \brief Callback that receives the world update event
    protected: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Calculate the score for the trays given the objects in them
    protected: void ScoreTrays();

    /// \brief Calculate the score of a single tray given the objects
    protected: void ScoreTray(const ariac::Kit & goalKit, const ariac::Kit & currentKit);
    protected: physics::WorldPtr world;

    /// \brief Helper function for filling a Kit from a kit ROS message
    public: static void FillKitFromMsg(const osrf_gear::Kit &kitMsg, ariac::Kit &kit);

    /// \brief Pointer to this node for publishing/subscribing
    protected: transport::NodePtr node;

    /// \brief Pointer to the update event connection.
    protected: event::ConnectionPtr updateConnection;

    /// \brief ROS node handle
    protected: ros::NodeHandle *rosNode;

    /// \brief Subscriber for goal messages
    protected: ros::Subscriber goalSub;

    /// \brief Subscriber for tray state messages
    protected: ros::Subscriber traySub;

    /// \brief Callback for receiving goal message
    public: void OnGoalReceived(const osrf_gear::Goal::ConstPtr & goalMsg);

    /// \brief Callback for receiving tray state message
    public: void OnTrayInfo(const osrf_gear::Kit::ConstPtr & trayMsg);

    /// \brief The goal state of the kits on the different trays
    protected: std::map<std::string, ariac::Kit> assignedKits;

    /// \brief The current state of the kits on the different trays
    protected: std::map<std::string, ariac::Kit> currentKits;

    /// \brief Mutex for protecting the assigned kits
    protected: mutable boost::mutex assignedKitsMutex;

    /// \brief Flag for signalling new tray info to process
    protected: bool newTrayInfo;

    /// \brief Flag for signalling new goal to process
    protected: bool newGoal;

    /// \brief Mutex for protecting the current kits
    protected: mutable boost::mutex currentKitsMutex;

    public: typedef struct ScoringParameters
               {
                 double objectPresence = 1.0;
                 double objectPosition = 0.0;
                 double objectOrientation = 1.0;

                 // Bonus when all objects in the tray: fator * (number of objects)
                 double allObjectsBonusFactor = 1.0;
                 // Acceptable distance in meters to object's target position
                 double distanceThresh = 0.03;
               } ScoringParameters;
    protected: ScoringParameters scoringParameters;
  };
}
#endif

