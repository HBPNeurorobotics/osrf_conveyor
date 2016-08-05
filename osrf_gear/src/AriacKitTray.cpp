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
  std::cout << "Assigned new kit." << std::endl;
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
  std::cout << "[" << this->trayID << "] Comparing the " << numAssignedObjects <<
    " assigned objects with the current " <<
    this->currentKit.objects.size() << " objects" << std::endl;

  std::vector<ariac::KitObject> remainingAssignedObjects(assignedKit.objects);
  std::map<std::string, unsigned int> currentObjectTypeCount;

  std::cout << "[" << this->trayID << "] Checking object counts" << std::endl;
  bool assignedObjectsMissing = false;
  for (auto & value : this->assignedObjectTypeCount)
  {
    auto assignedObjectType = value.first;
    auto assignedObjectCount = value.second;
    auto currentObjectCount =
      std::count_if(this->currentKit.objects.begin(), currentKit.objects.end(),
        [assignedObjectType](ariac::KitObject k) {return k.type == assignedObjectType;});
    std::cout << "[" << this->trayID << "] Found " << currentObjectCount <<
      " objects of type '" << assignedObjectType << "'" << std::endl;
    score.partPresence +=
      std::min(long(assignedObjectCount), currentObjectCount) * scoringParameters.objectPresence;
    if (currentObjectCount < assignedObjectCount)
    {
      assignedObjectsMissing = true;
    }
  }
  if (!assignedObjectsMissing)
  {
    std::cout << "[" << this->trayID << "] All objects on tray" << std::endl;
    score.allPartsBonus += scoringParameters.allObjectsBonusFactor * numAssignedObjects;
  }

  std::cout << "[" << this->trayID << "] Checking object poses" << std::endl;
  for (const auto & currentObject : this->currentKit.objects)
  {
    for (auto it = remainingAssignedObjects.begin(); it != remainingAssignedObjects.end(); ++it)
    {
      auto assignedObject = *it;
      if (assignedObject.type != currentObject.type)
        continue;

      math::Vector3 posnDiff = assignedObject.pose.CoordPositionSub(currentObject.pose);
      posnDiff.z = 0;
      if (posnDiff.GetLength() > scoringParameters.distanceThresh)
        continue;
      std::cout << "[" << this->trayID << "] Object of type '" << currentObject.type <<
        "' in the correct position" << std::endl;
      score.partPose += scoringParameters.objectPosition;

      // TODO: check orientation
      score.partPose += scoringParameters.objectOrientation;
      //std::cout << "Object '" << currentObject.type << "' in the correct position" << std::endl;

      // Once a match is found, don't permit it to be matched again
      remainingAssignedObjects.erase(it);
      break;
    }
  }

  if (remainingAssignedObjects.empty())
  {
    score.isComplete = true;
  }

  this->currentScore = score;
  this->currentScoringParameters = scoringParameters;
  this->kitStateChanged = false;
  this->assignedKitChanged = false;
  return score;
}
