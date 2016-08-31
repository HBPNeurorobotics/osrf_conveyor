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
#include <osrf_gear/KitTray.h>
#include "osrf_gear/VacuumGripperState.h"

/// \brief A scorer for the ARIAC game.
class AriacScorer
{
  /// \brief Constructor.
  public: AriacScorer();

  /// \brief Destructor.
  public: virtual ~AriacScorer();

  /// \brief Update the scorer.
  public: void Update(double timeStep = 0.0);

  /// \brief Get the current score.
  /// \param[in] The time in seconds since the last update.
  /// \return The score for the game.
  public: ariac::GameScore GetGameScore();

  /// \brief Get the score of the current goal.
  /// \return True if the goal is complete.
  public: bool IsCurrentGoalComplete();

  /// \brief Get the score of the current goal.
  /// \return The score for the goal.
  public: ariac::GoalScore GetCurrentGoalScore();

  /// \brief Assign a goal to process.
  /// \param[in] The goal.
  public: void AssignGoal(const ariac::Goal & goal);

  /// \brief Stop processing the current goal.
  /// \param[in] The time spent on the goal.
  /// \return The score for the goal.
  public: ariac::GoalScore UnassignCurrentGoal(double timeTaken = 0.0);

  /// \brief Calculate the score for a tray given the type of kit being built.
  public: ariac::TrayScore ScoreTray(const ariac::Kit & tray, const ariac::KitType_t kitType);

  /// \brief Calculate the score for the trays given the objects in them.
  protected: void ScoreCurrentGoal();

  /// \brief Helper function for filling a Kit from a kit ROS message.
  public: static void FillKitFromMsg(const osrf_gear::Kit & kitMsg, ariac::Kit & kit);

  /// \brief Callback for receiving goal message.
  public: void OnGoalReceived(const osrf_gear::Goal::ConstPtr & goalMsg);

  /// \brief Callback for receiving tray state message.
  public: void OnTrayInfoReceived(const osrf_gear::KitTray::ConstPtr & trayMsg);

  /// \brief Callback for receiving gripper state message.
  public: void OnGripperStateReceived(const osrf_gear::VacuumGripperState &stateMsg);

  /// \brief The trays to monitor the score of.
  protected: std::map<ariac::TrayID_t, ariac::KitTray> kitTrays;

  /// \brief Mutex for protecting the kit trays being monitored.
  protected: mutable boost::mutex kitTraysMutex;

  /// \brief Current goal being monitored.
  protected: ariac::Goal currentGoal;

  /// \brief Flag for signalling new tray info to process.
  protected: bool newTrayInfoReceived;

  /// \brief Flag for signalling new goal to process.
  protected: bool newGoalReceived;

  /// \brief Whether or not there is a travelling part in the gripper.
  protected: bool isPartTravelling = false;

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

