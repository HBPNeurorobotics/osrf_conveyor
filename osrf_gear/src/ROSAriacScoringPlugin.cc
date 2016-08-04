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

#include "ROSAriacScoringPlugin.hh"
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Quaternion.hh>
#include <osrf_gear/Goal.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <ignition/math/Matrix4.hh>

using namespace gazebo;
GZ_REGISTER_WORLD_PLUGIN(ROSAriacScoringPlugin)

/////////////////////////////////////////////////
ROSAriacScoringPlugin::ROSAriacScoringPlugin() : WorldPlugin()
{
}

/////////////////////////////////////////////////
ROSAriacScoringPlugin::~ROSAriacScoringPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void ROSAriacScoringPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
{
  this->world = _world;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosNode = new ros::NodeHandle("");
  this->goalSub = this->rosNode->subscribe(
    "/ariac/goals", 10, &ROSAriacScoringPlugin::OnGoalReceived, this);
  this->traySub = this->rosNode->subscribe(
    "/ariac/trays", 10, &ROSAriacScoringPlugin::OnTrayInfo, this);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&ROSAriacScoringPlugin::OnUpdate, this, _1));

  this->kitTrays.clear();
  gzdbg << "Scoring plugin loaded\n";
}

/////////////////////////////////////////////////
void ROSAriacScoringPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  boost::mutex::scoped_lock kitTraysLock(this->kitTraysMutex);

  if (this->newGoalReceived)
  {
    this->ProcessNewGoal();
  }

  if (this->newGoalReceived || this->newTrayInfoReceived)
  {
    this->ScoreTrays();
  }
  this->newGoalReceived = false;
  this->newTrayInfoReceived = false;
}

/////////////////////////////////////////////////
ariac::GoalScore ROSAriacScoringPlugin::ScoreTrays()
{
  ariac::GoalScore score;
  double total = 0;
  for (const auto & item : this->kitTrays)
  {
    auto tray = item.second;
    auto trayScore = tray.ScoreTray(this->scoringParameters);
    gzdbg << "Score from tray '" << item.first << "': " << trayScore.total() << "\n";
    score.trayScores.push_back(trayScore);
    total += trayScore.total();
  }
  std::cout << "Total score from trays: " << total << std::endl;
  return score;
}

/////////////////////////////////////////////////
void ROSAriacScoringPlugin::OnTrayInfo(const osrf_gear::Kit::ConstPtr & kitMsg)
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
void ROSAriacScoringPlugin::OnGoalReceived(const osrf_gear::Goal::ConstPtr & goalMsg)
{
  gzdbg << "Received a goal\n";
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
void ROSAriacScoringPlugin::ProcessNewGoal()
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
  for (const auto & item : this->kitTrays)
  {
    auto tray = item.second;
    tray.AddTrayGoalOutlines(this->world);
  }
}

/////////////////////////////////////////////////
void ROSAriacScoringPlugin::FillKitFromMsg(const osrf_gear::Kit &kitMsg, ariac::Kit &kit)
{
  kit.objects.clear();
  for (const auto & objMsg : kitMsg.objects)
  {
    ariac::KitObject obj;
    obj.type = ariac::DetermineModelType(objMsg.type);
    geometry_msgs::Point p = objMsg.pose.position;
    geometry_msgs::Quaternion o = objMsg.pose.orientation;
    obj.pose = math::Pose(math::Vector3(p.x, p.y, p.z), math::Quaternion(o.x, o.y, o.z, o.w));
    kit.objects.push_back(obj);
  }
}
