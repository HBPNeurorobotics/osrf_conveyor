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

#include <string>

#include <gazebo/math/Pose.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Quaternion.hh>

#include "osrf_gear/AriacScorer.h"

/////////////////////////////////////////////////
AriacScorer::AriacScorer()
{
}

/////////////////////////////////////////////////
AriacScorer::~AriacScorer()
{
}

/////////////////////////////////////////////////
ariac::GameScore AriacScorer::GetGameScore()
{
  return this->gameScore;
}

/////////////////////////////////////////////////
ariac::GoalScore AriacScorer::GetCurrentGoalScore()
{
  return *this->goalScore;
}

/////////////////////////////////////////////////
void AriacScorer::Update()
{
  boost::mutex::scoped_lock kitTraysLock(this->kitTraysMutex);

  if (this->newGoalReceived)
  {
    this->ProcessNewGoal();
  }

  if (this->newGoalReceived || this->newTrayInfoReceived)
  {
    this->ScoreCurrentGoal();
  }
  this->newGoalReceived = false;
  this->newTrayInfoReceived = false;
}

/////////////////////////////////////////////////
bool AriacScorer::IsCurrentGoalComplete()
{
  return this->goalScore->isComplete();
}

/////////////////////////////////////////////////
void AriacScorer::ScoreCurrentGoal()
{
  for (const auto & item : this->currentGoal.kits)
  {
    auto trayID = item.first;
    // Only count the trays that are part of this goal.
    if (this->kitTrays.find(trayID) != this->kitTrays.end())
    {
      auto tray = this->kitTrays[trayID];
      auto trayScore = tray.ScoreTray(this->scoringParameters);
      ROS_DEBUG_STREAM("Score from tray '" << trayID << "': " << trayScore.total());
      this->goalScore->trayScores[trayID] = trayScore;
    }
  }
}

/////////////////////////////////////////////////
void AriacScorer::OnTrayInfoReceived(const osrf_gear::Kit::ConstPtr & kitMsg)
{
  boost::mutex::scoped_lock kitTraysLock(this->kitTraysMutex);

  // Get the ID of the tray that the message is from.
  std::string trayID = kitMsg->tray.data;

  if (this->kitTrays.find(trayID) == this->kitTrays.end())
  {
    // This is the first time we've heard from this tray: initialize it.
    this->kitTrays[trayID] = ariac::KitTray(trayID);
  }

  // Update the state of the tray.
  // TODO: this should be moved outside of the callback
  // Do this even if the tray isn't part of the current goal because maybe it
  // will be part of future goals.
  this->newTrayInfoReceived = true;
  ariac::Kit kitState;
  FillKitFromMsg(*kitMsg, kitState);
  this->kitTrays[trayID].UpdateKitState(kitState);
}

/////////////////////////////////////////////////
void AriacScorer::OnGoalReceived(const osrf_gear::Goal::ConstPtr & goalMsg)
{
  ROS_DEBUG("Received a goal");
  this->newGoalReceived = true;

  ariac::Goal goal;
  goal.goalID = goalMsg->goal_id.data;
  for (const auto & kitMsg : goalMsg->kits)
  {
    std::string trayID = kitMsg.tray.data;
    ariac::Kit assignedKit;
    FillKitFromMsg(kitMsg, assignedKit);
    goal.kits[trayID] = assignedKit;
  }
  this->newGoal = goal;
}

/////////////////////////////////////////////////
void AriacScorer::ProcessNewGoal()
{
  // TODO: store previous goal

  ariac::GoalID_t goalID = this->newGoal.goalID;
  if (this->gameScore.goalScores.find(goalID) == this->gameScore.goalScores.end())
  {
    // This is a previously unseen goal: start scoring from scratch
    this->gameScore.goalScores[goalID] = ariac::GoalScore();
  }
  this->goalScore = &this->gameScore.goalScores[goalID];

  // Assign the new goal
  for (const auto & item : this->newGoal.kits)
  {
    auto trayID = item.first;
    auto assignedKit = item.second;

    if (this->kitTrays.find(trayID) == this->kitTrays.end())
    {
      // This is a previously unmonitored tray: initialize it
      this->kitTrays[trayID] = ariac::KitTray(trayID, assignedKit);
    }
    else
    {
      this->kitTrays[trayID].AssignKit(assignedKit);
    }
  }

  // Add the outlines of the goal kits
  // TODO: remove these when goals change
  // TODO: do this using calls to gazebo
  /*
  for (const auto & item : this->kitTrays)
  {
    auto tray = item.second;
    tray.AddTrayGoalOutlines(this->world);
  }
  */
  this->currentGoal = this->newGoal;
}

/////////////////////////////////////////////////
void AriacScorer::FillKitFromMsg(const osrf_gear::Kit &kitMsg, ariac::Kit &kit)
{
  kit.objects.clear();
  for (const auto & objMsg : kitMsg.objects)
  {
    ariac::KitObject obj;
    obj.type = ariac::DetermineModelType(objMsg.type);
    geometry_msgs::Point p = objMsg.pose.position;
    geometry_msgs::Quaternion o = objMsg.pose.orientation;
    gazebo::math::Vector3 objPosition(p.x, p.y, p.z);
    gazebo::math::Quaternion objOrientation(o.w, o.x, o.y, o.z);
    obj.pose = gazebo::math::Pose(objPosition, objOrientation);
    kit.objects.push_back(obj);
  }
}
