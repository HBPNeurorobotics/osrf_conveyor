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
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj TrayScore object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const TrayScore &_obj)
    {
      _out << "<tray_score " << _obj.trayID << ">" << std::endl;
      _out << "Total score: [" << _obj.total() << "]" << std::endl;
      _out << "Complete: [" << (_obj.isComplete ? "true" : "false") << "]" << std::endl;
      _out << "Part presence score: [" << _obj.partPresence << "]" << std::endl;
      _out << "All parts bonus: [" << _obj.allPartsBonus << "]" << std::endl;
      _out << "Part pose score: [" << _obj.partPose << "]" << std::endl;
      _out << "</tray_score>" << std::endl;
      return _out;
    }
    public: TrayID_t trayID;
            double partPresence = 0.0;
            double allPartsBonus = 0.0;
            double partPose = 0.0;
            bool isComplete = false;

            /// \brief Calculate the total score.
            double total() const
            {
              return partPresence + allPartsBonus + partPose;
            }
  };

  /// \brief The score of a goal.
  class GoalScore
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj GoalScore object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const GoalScore &_obj)
    {
      _out << "<goal_score " << _obj.goalID << ">" << std::endl;
      _out << "Total score: [" << _obj.total() << "]" << std::endl;
      _out << "Time taken: [" << _obj.timeTaken << "]" << std::endl;
      _out << "Complete: [" << (_obj.isComplete() ? "true" : "false") << "]" << std::endl;
      for (const auto & item : _obj.trayScores)
      {
        _out << item.second << std::endl;
      }
      _out << "</goal_score>" << std::endl;
      return _out;
    }

    /// \brief Mapping between tray IDs and scores.
    public: std::map<TrayID_t, TrayScore> trayScores;

            /// \brief ID of the goal being scored.
            GoalID_t goalID;

            /// \brief Time in seconds spend on the goal.
            double timeTaken = 0.0;

            /// \brief Calculate if the goal is complete.
            /// \return True if all trays are complete.
            ///   Will return false if there are no trays in the goal.
            bool isComplete() const
            {
              bool isGoalComplete = !this->trayScores.empty();
              for (const auto & item : this->trayScores)
              {
                isGoalComplete &= item.second.isComplete;
                if (!isGoalComplete)
                {
                  break;
                }
              }
              return isGoalComplete;
            };

            /// \brief Calculate the total score.
            double total() const
            {
              double total = 0.0;
              for (const auto & item : this->trayScores)
              {
                total += item.second.total();
              }
              return total;
            };
  };

  /// \brief The score of a competition run.
  class GameScore
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj GameScore object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const GameScore &_obj)
    {
      _out << "<game_score>" << std::endl;
      _out << "Total score: [" << _obj.total() << "]" << std::endl;
      _out << "Total process time: [" << _obj.totalProcessTime << "]" << std::endl;
      _out << "Part travel time: [" << _obj.partTravelTime << "]" << std::endl;
      for (const auto & item : _obj.goalScores)
      {
        _out << item.second << std::endl;
      }
      _out << "</game_score>" << std::endl;
      return _out;
    }

    public: double totalProcessTime = 0.0;
            double partTravelTime = 0.0;
            double planningTime = 0.0;
            double partTravelDistance = 0.0;
            double manipulatorTravelDistance = 0.0;

            // The score of each of the goals during the game.
            std::map<GoalID_t, GoalScore> goalScores;

            /// \brief Calculate the total score.
            double total() const
            {
              double total = 0;
              total += totalProcessTime;
              total += partTravelTime;
              total += planningTime;
              total += partTravelDistance;
              total += manipulatorTravelDistance;

              for (const auto & item : this->goalScores)
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

    // Acceptable distance in meters to object's target position.
    // The measured distance is between the center of the model and its target,
    // projected onto the tray.
    public: double distanceThresh = 0.03;

    // Acceptable difference in radians to object's target orientation.
    // The measured difference is from a top-down view of the tray, but only if
    // the quaternions are aligned.
    public: double orientationThresh = 0.1;
  };

  /// \brief Determine the type of a gazebo model from its name
  TrayID_t DetermineModelType(const std::string &modelName)
  {
    TrayID_t modelType(modelName);

    // Trim namespaces
    size_t index = modelType.find_last_of('|');
    modelType = modelType.substr(index + 1);

    // Trim trailing underscore and number caused by inserting multiple of the same model
    index = modelType.find_last_not_of("0123456789");
    if (modelType[index] == '_' && index > 1)
    {
      modelType = modelType.substr(0, index);
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
      for (const auto & obj : _kit.objects)
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
      return this->startTime < _goal.startTime;
    }

    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _goal Goal to output.
    /// \return The output stream.
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Goal &_goal)
    {
      _out << "<Goal>" << std::endl;
      _out << "Start time: [" << _goal.startTime << "]" << std::endl;
      _out << "Allowed time: [" << _goal.allowedTime << "]" << std::endl;
      _out << "Kits:" << std::endl;
      for (const auto & item : _goal.kits)
      {
        _out << item.second << std::endl;
      }
      _out << "</goal>" << std::endl;

      return _out;
    }

    /// \brief The ID of this goal.
    public: GoalID_t goalID;

    /// \brief Simulation time in which the goal should be triggered.
    public: double startTime;

    /// \brief Simulation time in seconds permitted for the goal to be
    /// completed before cancelling it. Infinite by default.
    public: double allowedTime;

    /// \brief A goal is composed of multiple kits assigned to specific trays.
    public: std::map<TrayID_t, Kit> kits;

    /// \brief Simulation time in seconds spent on this goal.
    public: double timeTaken;
  };
}
#endif
