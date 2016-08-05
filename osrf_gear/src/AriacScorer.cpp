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
AriacScorer::AriacScorer(ros::NodeHandle & /*node*/)
{
  // TODO: connect score publisher
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
  return this->goalScore;
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
void AriacScorer::ScoreCurrentGoal()
{
  double total = 0;
  for (const auto & item : this->kitTrays)
  {
    auto tray = item.second;
    auto trayScore = tray.ScoreTray(this->scoringParameters);
    ROS_INFO_STREAM("Score from tray '" << item.first << "': " << trayScore.total());
    this->goalScore.trayScores[item.first] = trayScore;
    total += trayScore.total();
  }
  ROS_INFO_STREAM("Total score from trays: " << total);
}

/////////////////////////////////////////////////
void AriacScorer::OnTrayInfoReceived(const osrf_gear::Kit::ConstPtr & kitMsg)
{
  boost::mutex::scoped_lock kitTraysLock(this->kitTraysMutex);

  // Get the ID of the tray that the message is from
  std::string trayID = kitMsg->tray.data;

  if (this->kitTrays.find(trayID) == this->kitTrays.end())
  {
    // The tray is not part of the current goal - ignore it
    return;
  }

  // Update the state of the tray
  this->newTrayInfoReceived = true;
  ariac::Kit kitState;
  FillKitFromMsg(*kitMsg, kitState);
  this->kitTrays[trayID].UpdateKitState(kitState);
}

/////////////////////////////////////////////////
void AriacScorer::OnGoalReceived(const osrf_gear::Goal::ConstPtr & goalMsg)
{
  ROS_INFO("Received a goal");
  this->newGoalReceived = true;

  ariac::Goal goal;
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
  bool goalAssigned = !this->kitTrays.empty();
  if (goalAssigned)
  {
    // Add the score of the previous goal to the total
    this->gameScore.goalScores.push_back(this->goalScore);
  }
  this->goalScore = ariac::GoalScore();

  // Assign the new goal
  // TODO: don't wipe state of trays that are in this goal
  this->kitTrays.clear();
  for (const auto & item : this->newGoal.kits)
  {
    auto trayID = item.first;
    auto assignedKit = item.second;
    this->kitTrays[trayID] = ariac::KitTray(trayID, assignedKit);
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
    obj.pose = gazebo::math::Pose(
      gazebo::math::Vector3(p.x, p.y, p.z), gazebo::math::Quaternion(o.x, o.y, o.z, o.w));
    kit.objects.push_back(obj);
  }
}
