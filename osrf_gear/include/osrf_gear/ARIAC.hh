/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef _ARIAC_HH_
#define _ARIAC_HH_

#include <ostream>
#include <map>
#include <string>
#include <vector>

#include <gazebo/gazebo.hh>

namespace ariac
{
  using namespace gazebo;

  typedef std::string TrayID_t;
  typedef std::string GoalID_t;

  /// \brief The score of a tray.
  class TrayScore
  {
    public: double partPresence = 0.0;
            double allPartsBonus = 0.0;
            double partPose = 0.0;
            bool isComplete = false;

            /// \brief Calculate the total score.
            double total()
            {
              return partPresence + allPartsBonus + partPose;
            }
  };

  /// \brief The score of a goal.
  class GoalScore
  {
    /// \brief Mapping between tray IDs and scores
    public: std::map<TrayID_t, TrayScore> trayScores;

            /// \brief Calculate if the goal is complete.
            /// \return True if all trays are complete.
            ///   Will return false if there are no trays in the goal.
            bool isComplete()
            {
              bool isComplete = !this->trayScores.empty();
              for (auto item : this->trayScores)
              {
                isComplete &= item.second.isComplete;
                if (!isComplete)
                {
                  break;
                }
              }
              return isComplete;
            };

            /// \brief Calculate the total score.
            double total()
            {
              double total = 0;
              for (auto item : this->trayScores)
              {
                total += item.second.total();
              }
              return total;
            };
  };

  /// \brief The score of a competition run.
  class GameScore
  {
    public: double totalProcessTime = 0.0;
            double partTravelTime = 0.0;
            double planningTime = 0.0;
            double partTravelDistance = 0.0;
            double manipulatorTravelDistance = 0.0;

            // The score of each of the goals during the game.
            std::map<GoalID_t, GoalScore> goalScores;

            /// \brief Calculate the total score.
            double total()
            {
              double total = 0;
              total += totalProcessTime;
              total += partTravelTime;
              total += planningTime;
              total += partTravelDistance;
              total += manipulatorTravelDistance;

              for (auto item : this->goalScores)
              {
                total += item.second.total();
              }
              return total;
            };
  };

  /// \brief The parameters used for scoring the competition.
  // TODO: this should have a different data type
  class ScoringParameters
  {
    /// \brief Equality comparison operator.
    /// \param[in] sp1 First parameters to compare.
    /// \param[in] sp2 Second parameters to compare.
    /// \return True if sp1 == sp2.
    public: friend bool operator==(const ScoringParameters &sp1, const ScoringParameters &sp2)
    {
      return (
        sp1.objectPresence == sp2.objectPresence &&
        sp1.objectPosition == sp2.objectPosition &&
        sp1.objectOrientation == sp2.objectOrientation &&
        sp1.allObjectsBonusFactor == sp2.allObjectsBonusFactor &&
        sp1.distanceThresh == sp2.distanceThresh);
    }

    /// \brief Inequality comparison operator.
    /// \param[in] sp1 First parameters to compare.
    /// \param[in] sp2 Second parameters to compare.
    /// \return True if sp1 != sp2.
    public: friend bool operator!=(const ScoringParameters &sp1, const ScoringParameters &sp2)
    {
      return !(sp1 == sp2);
    }

    public: double objectPresence = 1.0;
    public: double objectPosition = 0.0;
    public: double objectOrientation = 1.0;

    // Bonus when all objects in the tray: factor * (number of objects)
    public: double allObjectsBonusFactor = 1.0;

    // Acceptable distance in meters to object's target position
    public: double distanceThresh = 0.03;
  };

  /// \brief Determine the type of a gazebo model from its name
  TrayID_t DetermineModelType(const std::string &modelName)
  {
    TrayID_t modelType(modelName);

    // Trim namespaces
    size_t index = modelType.find_last_of(':');
    modelType = modelType.substr(index + 1);

    // Trim trailing underscore and number caused by inserting multiple of the same model
    index = modelName.find_last_not_of("0123456789");
    if (modelName[index] == '_' && index > 1)
    {
      modelType = modelName.substr(0, index);
    }

    return modelType;
  }

  /// \brief Class to store information about each object contained in a kit.
  class KitObject
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj Kit object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const KitObject &_obj)
    {
      _out << "<object>" << std::endl;
      _out << "Type: [" << _obj.type << "]" << std::endl;
      _out << "Pose: [" << _obj.pose << "]" << std::endl;
      _out << "</object>" << std::endl;
      return _out;
    }

    /// \brief Object type.
    public: std::string type;

    /// \brief Pose in which the object should be placed.
    public: math::Pose pose;

  };

  /// \brief Class to store information about a kit.
  class Kit
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _kit kit to output.
    /// \return The output stream.
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Kit &_kit)
    {
      _out << "<kit>";
      for (auto obj : _kit.objects)
        _out << std::endl << obj;
      _out << std::endl << "</kit>" << std::endl;

      return _out;
    }

    /// \brief A kit is composed by multiple objects.
    public: std::vector<KitObject> objects;
  };

  /// \brief Class to store information about a goal.
  class Goal
  {
    /// \brief Less than operator.
    /// \param[in] _goal Other goal to compare.
    /// \return True if this < _goal.
    public: bool operator<(const Goal &_goal) const
    {
      return this->time < _goal.time;
    }

    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _goal Goal to output.
    /// \return The output stream.
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Goal &_goal)
    {
      _out << "<Goal>" << std::endl;
      _out << "Time: [" << _goal.time << "]" << std::endl;
      _out << "Kits:" << std::endl;
      for (auto item : _goal.kits)
      {
        _out << "<tray>" << item.first << "</tray>" << std::endl;
        _out << item.second << std::endl;
      }
      _out << "</goal>" << std::endl;

      return _out;
    }

    /// \brief The ID of this goal.
    public: GoalID_t goalID;

    /// \brief Simulation time in which the goal should be triggered.
    public: double time;

    /// \brief A goal is composed of multiple kits assigned to specific trays.
    public: std::map<TrayID_t, Kit> kits;
  };
}
#endif
