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

#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Quaternion.hh>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <ignition/math/Matrix4.hh>
#include "ROSAriacKitTray.hh"

using namespace ariac;
using namespace gazebo;

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
  gzdbg << "Assigned new kit.\n";
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
  gzdbg << "Updating kit.\n";
  this->currentKit = kit;
  this->kitStateChanged = true;
}

/////////////////////////////////////////////////
bool KitTray::AddTrayGoalOutlines(physics::WorldPtr world)
{
  gzdbg << "Adding tray goal outlines from " << this->trayID << "\n";

  auto trayFrame = world->GetEntity(this->trayID);

  if (!trayFrame)
  {
    gzdbg << "Cannot find frame for tray: " << this->trayID << ". Not adding tray goal outlines.\n";
    return false;
  }

  ignition::math::Pose3d trayPose;
  if (trayFrame->HasType(physics::Base::LINK) || trayFrame->HasType(physics::Base::MODEL))
  {
    trayPose = trayFrame->GetWorldPose().Ign();
  }

  unsigned int objectID = 0;
  for (auto obj : this->assignedKit.objects)
  {
    std::ostringstream newModelStr;
    std::string modelType = obj.type + "_outline";
    // Give each object a unique name so that their names don't clash when their models are
    // added during the same sim step. Use the tray ID in the name so they don't clash with
    // existing models on other trays.
    std::string modelName = this->trayID + "_part_ " + std::to_string(objectID++) + "_" + modelType;

    ignition::math::Matrix4d transMat(trayPose);
    ignition::math::Matrix4d pose_local(obj.pose.Ign());
    obj.pose = (transMat * pose_local).Pose();

    newModelStr <<
      "<sdf version='" << SDF_VERSION << "'>\n"
      "  <include>\n"
      "    <pose>" << obj.pose << "</pose>\n"
      "    <name>" << modelName << "</name>\n"
      "    <uri>model://" << modelType << "</uri>\n"
      "  </include>\n"
      "</sdf>\n";
    gzdbg << "Inserting model: " << modelName << "\n";
    world->InsertModelString(newModelStr.str());

    // TODO: add a joint to the tray instead of using static models
  }
  return true;
}

/////////////////////////////////////////////////
double KitTray::ScoreTray(const ScoringParameters & scoringParameters)
{
  // If nothing has changed, return the previously calculated score
  // TODO: what if the scoring parameters have changed?
  if (!(this->kitStateChanged || this->assignedKitChanged))
  {
    return this->currentScore;
  }

  double score = 0;
  auto numAssignedObjects = this->assignedKit.objects.size();
  gzdbg << "Comparing the " << numAssignedObjects << " assigned objects with the current " << \
    this->currentKit.objects.size() << " objects\n";

  std::vector<ariac::KitObject> remainingAssignedObjects(assignedKit.objects);
  std::map<std::string, unsigned int> currentObjectTypeCount;

  gzdbg << "Checking object counts\n";
  bool assignedObjectsMissing = false;
  for (auto & value : this->assignedObjectTypeCount)
  {
    auto assignedObjectType = value.first;
    auto assignedObjectCount = value.second;
    auto currentObjectCount =
      std::count_if(this->currentKit.objects.begin(), currentKit.objects.end(),
        [assignedObjectType](ariac::KitObject k) {return k.type == assignedObjectType;});
    gzdbg << "Found " << currentObjectCount << " objects of type '" << assignedObjectType << "'\n";
    score += std::min(long(assignedObjectCount), currentObjectCount) * scoringParameters.objectPresence;
    if (currentObjectCount < assignedObjectCount)
    {
      assignedObjectsMissing = true;
    }
  }
  if (!assignedObjectsMissing)
  {
    gzdbg << "All objects on tray\n";
    score += scoringParameters.allObjectsBonusFactor * numAssignedObjects;
  }

  gzdbg << "Checking object poses\n";
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
      gzdbg << "Object of type '" << currentObject.type << "' in the correct position\n";
      score += scoringParameters.objectPosition;

      // TODO: check orientation
      score += scoringParameters.objectOrientation;
      //gzdbg << "Object '" << currentObject.type << "' in the correct position\n";

      // Once a match is found, don't permit it to be matched again
      remainingAssignedObjects.erase(it);
      break;
    }
  }

  this->currentScore = score;
  this->kitStateChanged = false;
  this->assignedKitChanged = false;
  return score;
}
