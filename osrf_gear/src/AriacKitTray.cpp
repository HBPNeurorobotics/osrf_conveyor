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

#include "osrf_gear/AriacKitTray.h"

#include <iostream>
#include <vector>

#include <ros/console.h>

using namespace ariac;

/////////////////////////////////////////////////
KitTray::KitTray()
{
}

/////////////////////////////////////////////////
KitTray::~KitTray()
{
}

/////////////////////////////////////////////////
KitTray::KitTray(std::string _trayID, const Kit & _assignedKit)
  : trayID(_trayID)
{
  this->AssignKit(_assignedKit);
}

/////////////////////////////////////////////////
void KitTray::AssignKit(const Kit & kit)
{
  ROS_DEBUG_STREAM("Assigned new kit.");
  this->assignedKit = kit;
  this->assignedKitChanged = true;

  // Count the number of each type of object
  this->assignedObjectTypeCount.clear();
  for (const auto & obj : kit.objects)
  {
    if (this->assignedObjectTypeCount.find(obj.type) == this->assignedObjectTypeCount.end())
    {
      this->assignedObjectTypeCount[obj.type] = 0;
    }
    this->assignedObjectTypeCount[obj.type] += 1;
  }

}

/////////////////////////////////////////////////
void KitTray::UpdateKitState(const Kit & kit)
{
  this->currentKit = kit;
  this->kitStateChanged = true;
}

/////////////////////////////////////////////////
TrayScore KitTray::ScoreTray(const ScoringParameters & scoringParameters)
{
  bool scoringParametersChanged = this->currentScoringParameters != scoringParameters;

  // If nothing has changed, return the previously calculated score
  if (!(this->kitStateChanged || this->assignedKitChanged || scoringParametersChanged))
  {
    return this->currentScore;
  }

  TrayScore score;
  auto numAssignedObjects = this->assignedKit.objects.size();
  ROS_DEBUG_STREAM("[" << this->trayID << "] Comparing the " << numAssignedObjects <<
    " assigned objects with the current " <<
    this->currentKit.objects.size() << " objects");

  // Keep track of which assigned objects have already been 'matched' to one on the tray.
  // This is to prevent multiple objects being close to a single target pose both scoring points.
  std::vector<ariac::KitObject> remainingAssignedObjects(assignedKit.objects);

  ROS_DEBUG_STREAM("[" << this->trayID << "] Checking object counts");
  std::map<std::string, unsigned int> currentObjectTypeCount;
  bool assignedObjectsMissing = false;
  for (auto & value : this->assignedObjectTypeCount)
  {
    auto assignedObjectType = value.first;
    auto assignedObjectCount = value.second;
    auto currentObjectCount =
      std::count_if(this->currentKit.objects.begin(), currentKit.objects.end(),
        [assignedObjectType](ariac::KitObject k) {return k.type == assignedObjectType;});
    ROS_DEBUG_STREAM("[" << this->trayID << "] Found " << currentObjectCount <<
      " objects of type '" << assignedObjectType << "'");
    score.partPresence +=
      std::min(long(assignedObjectCount), currentObjectCount) * scoringParameters.objectPresence;
    if (currentObjectCount < assignedObjectCount)
    {
      assignedObjectsMissing = true;
    }
  }
  if (!assignedObjectsMissing)
  {
    ROS_DEBUG_STREAM("[" << this->trayID << "] All objects on tray");
    score.allPartsBonus += scoringParameters.allObjectsBonusFactor * numAssignedObjects;
  }

  ROS_DEBUG_STREAM("[" << this->trayID << "] Checking object poses");
  for (const auto & currentObject : this->currentKit.objects)
  {
    for (auto it = remainingAssignedObjects.begin(); it != remainingAssignedObjects.end(); ++it)
    {
      // Only check poses of parts of the same type
      auto assignedObject = *it;
      if (assignedObject.type != currentObject.type)
        continue;

      // Check the position of the object (ignoring orientation)
      math::Vector3 posnDiff = assignedObject.pose.CoordPositionSub(currentObject.pose);
      posnDiff.z = 0;
      if (posnDiff.GetLength() > scoringParameters.distanceThresh)
        continue;
      ROS_DEBUG_STREAM("[" << this->trayID << "] Object of type '" << currentObject.type <<
        "' in the correct position");
      score.partPose += scoringParameters.objectPosition;

      // Check the orientation of the object
      math::Quaternion objOrientation = currentObject.pose.rot;
      math::Quaternion goalOrientation = assignedObject.pose.rot;

      // If the quaternions represent the same orientation, q1 = +-q2 => q1.dot(q2) = 1
      double orientationDiff = std::abs(objOrientation.Dot(goalOrientation));
      if (orientationDiff < (1.0 - scoringParameters.orientationThresh))
        continue;
      ROS_DEBUG_STREAM("[" << this->trayID << "] Object of type '" << currentObject.type <<
        "' in the correct orientation");
      score.partPose += scoringParameters.objectOrientation;

      // Once a match is found, don't permit it to be matched again
      remainingAssignedObjects.erase(it);
      break;
    }
  }

  // Check if all assigned objects have been matched to one on the tray
  if (remainingAssignedObjects.empty())
  {
    score.isComplete = true;
    if (this->currentScore.isComplete != score.isComplete)
    {
      //FIXME: there's a bug here. it's not maintaining its state.
      ROS_INFO_STREAM("Tray complete: " << this->trayID);
    }
  }

  this->currentScore = score;
  this->currentScoringParameters = scoringParameters;
  this->kitStateChanged = false;
  this->assignedKitChanged = false;
  return score;
}
