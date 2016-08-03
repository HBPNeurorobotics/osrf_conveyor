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
void ROSAriacScoringPlugin::Load(physics::WorldPtr /*_world*/, sdf::ElementPtr /*_sdf*/)
{
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

  this->currentKits.clear();
  this->assignedKits.clear();
  gzdbg << "Scoring plugin loaded\n";
}

/////////////////////////////////////////////////
void ROSAriacScoringPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  this->ScoreTrays();
}

/////////////////////////////////////////////////
void ROSAriacScoringPlugin::ScoreTrays()
{
  boost::mutex::scoped_lock assignedKitsLock(this->assignedKitsMutex);
  boost::mutex::scoped_lock currentKitsLock(this->currentKitsMutex);

  for (const auto & goalTray : this->assignedKits)
  {
    auto currentTray = this->currentKits.find(goalTray.first);
    if (currentTray == this->currentKits.end())
    {
      continue;
    }
    gzdbg << goalTray.second << "\n" << currentTray->second << "\n";
    this->ScoreTray(goalTray.second, currentTray->second);
  }
}

/////////////////////////////////////////////////
void ROSAriacScoringPlugin::ScoreTray(const ariac::Kit & goalKit, const ariac::Kit & currentKit)
{
  double score = 0;
  auto numGoalObjects = goalKit.objects.size();
  gzdbg << "Comparing the " << numGoalObjects << " goal objects with the current " << \
    currentKit.objects.size() << " objects\n";

  std::vector<ariac::KitObject> remainingGoalObjects;
  remainingGoalObjects.reserve(numGoalObjects);
  std::map<std::string, unsigned int> goalObjectTypeCount, currentObjectTypeCount;
  for (auto goalObject : goalKit.objects)
  {
    remainingGoalObjects.push_back(goalObject);
    if (goalObjectTypeCount.find(goalObject.type) == goalObjectTypeCount.end())
    {
      goalObjectTypeCount[goalObject.type] = 0;
      currentObjectTypeCount[goalObject.type] = 0;
    }
    goalObjectTypeCount[goalObject.type] += 1;
  }

  for (const auto & currentObject : currentKit.objects)
  {
    gzdbg << "Checking object: \n" << currentObject << "\n";
    for (auto it = remainingGoalObjects.begin(); it != remainingGoalObjects.end(); ++it)
    {
      bool objectTypeMatched = false;  // to ensure each object can only match by type once
      auto goalObject = *it;
      gzdbg << "Goal object: " << goalObject << "\n";
      if (goalObject.type != currentObject.type)
        continue;

      // Don't award more matches for occurance than goal objects of that type
      currentObjectTypeCount[currentObject.type] += 1;
      if (!objectTypeMatched &&
        currentObjectTypeCount[currentObject.type] <= goalObjectTypeCount[goalObject.type])
      {
        gzdbg << "Object match: " << goalObject.type << "\n";
        objectTypeMatched = true;
        score += this->scoringParameters.objectPresence;
      }
      else
      {
        gzdbg << "All objects of type '" << goalObject.type << "' have already been matched\n";
      }

      math::Vector3 posnDiff = goalObject.pose.CoordPositionSub(currentObject.pose);
      posnDiff.z = 0;
      if (posnDiff.GetLength() > this->scoringParameters.distanceThresh)
        continue;
      score += this->scoringParameters.objectPosition;

      /*
      double orientationDiff = std::abs(goalObject.pose.rot.dot(currentObject.pose.rot));
      if (rotnDiff.Something() > this->scoringParameters.orientationThres)
        break;
        */
      score += this->scoringParameters.objectOrientation;

      // Once a match is found, don't permit it to be matched again
      remainingGoalObjects.erase(it);
      break;
    }
  }
  // Check if all goal objects are somewhere on the tray
  bool goalObjectsMissing = false;
  for (auto &it : goalObjectTypeCount)
  {
    if (it.second != currentObjectTypeCount[it.first])
    {
      goalObjectsMissing = true;
      break;
    }
  }
  if (!goalObjectsMissing)
  {
    score += this->scoringParameters.allObjectsBonusFactor * numGoalObjects;
  }

  std::cout << score << std::endl;
}

/////////////////////////////////////////////////
void ROSAriacScoringPlugin::OnTrayInfo(const osrf_gear::Kit::ConstPtr & kitMsg)
{
  // Update the state of the tray
  //TODO: Only pay attention if kit is assigned
  boost::mutex::scoped_lock currentKitsLock(this->currentKitsMutex);

  std::string trayID = kitMsg->tray.data;
  ariac::Kit kitState;
  FillKitFromMsg(*kitMsg, kitState);
  this->currentKits[trayID] = kitState;
}

/////////////////////////////////////////////////
void ROSAriacScoringPlugin::OnGoalReceived(const osrf_gear::Goal::ConstPtr & goalMsg)
{
  // TODO: store previous goal
  gzdbg << "Received a goal\n";
  boost::mutex::scoped_lock assignedKitsLock(this->assignedKitsMutex);

  this->assignedKits.clear();
  for (const auto & kitMsg : goalMsg->kits)
  {
    std::string trayID = kitMsg.tray.data;
    ariac::Kit assignedKit;
    FillKitFromMsg(kitMsg, assignedKit);
    this->assignedKits[trayID] = assignedKit;
  }
}

/////////////////////////////////////////////////
void ROSAriacScoringPlugin::FillKitFromMsg(const osrf_gear::Kit &kitMsg, ariac::Kit &kit)
{
  kit.objects.clear();
  for (const auto & objMsg : kitMsg.objects)
  {
    ariac::KitObject obj;
    obj.type = objMsg.type;
    geometry_msgs::Point p = objMsg.pose.position;
    geometry_msgs::Quaternion o = objMsg.pose.orientation;
    obj.pose = math::Pose(math::Vector3(p.x, p.y, p.z), math::Quaternion(o.x, o.y, o.z, o.w));
    kit.objects.push_back(obj);
  }
}
