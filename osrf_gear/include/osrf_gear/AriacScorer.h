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

#include <map>
#include <string>

#include <ros/ros.h>

#include "osrf_gear/ARIAC.hh"
#include "osrf_gear/AriacKitTray.h"
#include <osrf_gear/Goal.h>
#include <osrf_gear/Kit.h>

/// \brief A scorer for the ARIAC game.
class AriacScorer
{
  /// \brief Constructor.
  public: AriacScorer(ros::NodeHandle & node);

  /// \brief Destructor.
  public: virtual ~AriacScorer();

  /// \brief Update the scorer.
  public: void Update();

  /// \brief Get the current score.
  /// \return The score for the game.
  public: ariac::GameScore GetGameScore();

  /// \brief Get the score of the current goal.
  /// \return The score for the goal.
  public: ariac::GoalScore GetCurrentGoalScore();

  /// \brief Process a new goal.
  protected: void ProcessNewGoal();

  /// \brief Calculate the score for the trays given the objects in them.
  protected: void ScoreCurrentGoal();

  /// \brief Helper function for filling a Kit from a kit ROS message.
  public: static void FillKitFromMsg(const osrf_gear::Kit &kitMsg, ariac::Kit &kit);

  /// \brief Callback for receiving goal message.
  public: void OnGoalReceived(const osrf_gear::Goal::ConstPtr & goalMsg);

  /// \brief Callback for receiving tray state message.
  public: void OnTrayInfoReceived(const osrf_gear::Kit::ConstPtr & trayMsg);

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

  /// \brief Pointer to the score of the current goal.
  protected: ariac::GoalScore* goalScore;

  /// \brief The score of the run.
  protected: ariac::GameScore gameScore;

};
#endif

