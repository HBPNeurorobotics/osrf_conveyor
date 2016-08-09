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

#include <algorithm>
#include <limits>
#include <mutex>
#include <ostream>
#include <string>
#include <vector>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/msgs/gz_string.pb.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>


#include "osrf_gear/ARIAC.hh"
#include "osrf_gear/ROSAriacTaskManagerPlugin.hh"
#include "osrf_gear/AriacScorer.h"
#include "osrf_gear/Goal.h"
#include "osrf_gear/Kit.h"
#include "osrf_gear/KitObject.h"

namespace gazebo
{
  /// \internal
  /// \brief Private data for the ROSAriacTaskManagerPlugin class.
  struct ROSAriacTaskManagerPluginPrivate
  {
    /// \brief World pointer.
    public: physics::WorldPtr world;

    /// \brief SDF pointer.
    public: sdf::ElementPtr sdf;

    /// \brief Collection of goals to announce.
    public: std::vector<ariac::Goal> goalsToAnnounce;

    /// \brief Collection of goals which have been announced but are not yet complete.
    /// The goal at the top of the stack is the active goal.
    public: std::stack<ariac::Goal> goalsInProgress;

    /// \brief A scorer to mange the game score.
    public: AriacScorer ariacScorer;

    /// \brief The current game score.
    public: ariac::GameScore currentGameScore;

    /// \brief ROS node handle.
    public: std::unique_ptr<ros::NodeHandle> rosnode;

    /// \brief Publishes a goal.
    public: ros::Publisher goalPub;

    /// \brief ROS subscriber for the tray states.
    public: ros::Subscriber trayInfoSub;

    /// \brief Publishes the Gazebo task state.
    public: ros::Publisher gazeboTaskStatePub;

    /// \brief Service that allows the user to start the competition.
    public: ros::ServiceServer teamStartServiceServer;

    /// \brief Transportation node.
    public: transport::NodePtr node;

    /// \brief Publisher for enabling the object population on the conveyor.
    public: transport::PublisherPtr populatePub;

    /// \brief Connection event.
    public: event::ConnectionPtr connection;

    /// \brief The time the last update was called.
    public: common::Time lastUpdateTime;

    /// \brief The time specified in the object is relative to this time.
    public: common::Time gameStartTime;

    /// \brief Pointer to the current state.
    public: std::string currentState = "init";

    /// \brief A mutex to protect currentState.
    public: std::mutex mutex;
  };
}

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(ROSAriacTaskManagerPlugin)

/////////////////////////////////////////////////
static void fillGoalMsg(const ariac::Goal &_goal,
                        osrf_gear::Goal &_msgGoal)
{
  _msgGoal.goal_id.data = _goal.goalID;
  for (const auto item : _goal.kits)
  {
    osrf_gear::Kit msgKit;
    msgKit.tray.data = item.first;
    for (const auto &obj : item.second.objects)
    {
      osrf_gear::KitObject msgObj;
      msgObj.type = obj.type;
      msgObj.pose.position.x = obj.pose.pos.x;
      msgObj.pose.position.y = obj.pose.pos.y;
      msgObj.pose.position.z = obj.pose.pos.z;
      msgObj.pose.orientation.x = obj.pose.rot.x;
      msgObj.pose.orientation.y = obj.pose.rot.y;
      msgObj.pose.orientation.z = obj.pose.rot.z;
      msgObj.pose.orientation.w = obj.pose.rot.w;

      // Add the object to the kit.
      msgKit.objects.push_back(msgObj);
    }
    _msgGoal.kits.push_back(msgKit);
  }
}

/////////////////////////////////////////////////
ROSAriacTaskManagerPlugin::ROSAriacTaskManagerPlugin()
  : dataPtr(new ROSAriacTaskManagerPluginPrivate)
{
}

/////////////////////////////////////////////////
ROSAriacTaskManagerPlugin::~ROSAriacTaskManagerPlugin()
{
  this->dataPtr->rosnode->shutdown();
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::Load(physics::WorldPtr _world,
  sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "ROSAriacTaskManagerPlugin world pointer is NULL");
  GZ_ASSERT(_sdf, "ROSAriacTaskManagerPlugin sdf pointer is NULL");
  this->dataPtr->world = _world;
  this->dataPtr->sdf = _sdf;

  std::string robotNamespace = "";
  if (_sdf->HasElement("robot_namespace"))
  {
    robotNamespace = _sdf->GetElement(
      "robot_namespace")->Get<std::string>() + "/";
  }

  std::string teamStartServiceName = "start";
  if (_sdf->HasElement("team_start_service_name"))
    teamStartServiceName = _sdf->Get<std::string>("team_start_service_name");

  std::string gazeboTaskStateTopic = "gazebo_task/state";
  if (_sdf->HasElement("gazebo_task_state_topic"))
    gazeboTaskStateTopic = _sdf->Get<std::string>("gazebo_task_state_topic");

  std::string conveyorActivationTopic = "activation_topic";
  if (_sdf->HasElement("conveyor_activation_topic"))
    conveyorActivationTopic = _sdf->Get<std::string>("conveyor_activation_topic");

  std::string goalsTopic = "goals";
  if (_sdf->HasElement("goals_topic"))
    goalsTopic = _sdf->Get<std::string>("goals_topic");

  // Parse the time.
  if (!_sdf->HasElement("goal"))
  {
    gzerr << "Unable to find a <goal> element" << std::endl;
    return;
  }

  unsigned int goalCount = 0;
  sdf::ElementPtr goalElem = _sdf->GetElement("goal");
  while (goalElem)
  {
    // Parse the start time.
    if (!goalElem->HasElement("start_time"))
    {
      gzerr << "Unable to find <start_time> element in <goal>. Ignoring" << std::endl;
      goalElem = goalElem->GetNextElement("goal");
      continue;
    }
    sdf::ElementPtr startTimeElement = goalElem->GetElement("start_time");
    double startTime = startTimeElement->Get<double>();

    // Parse the allowed completion time.
    double allowedTime = std::numeric_limits<double>::infinity();
    if (goalElem->HasElement("allowed_time"))
    {
      sdf::ElementPtr allowedTimeElement = goalElem->GetElement("allowed_time");
      allowedTime = allowedTimeElement->Get<double>();
    }

    // Parse the kits.
    if (!goalElem->HasElement("kit"))
    {
      gzerr << "Unable to find <kit> element in <goal>. Ignoring" << std::endl;
      goalElem = goalElem->GetNextElement("goal");
      continue;
    }

    // Store all kits for a goal.
    std::map<std::string, ariac::Kit> kits;

    sdf::ElementPtr kitElem = goalElem->GetElement("kit");
    while (kitElem)
    {
      // Check the validity of the kit.
      if (!kitElem->HasElement("object"))
      {
        gzerr << "Unable to find <object> element in <kit>. Ignoring"
              << std::endl;
        kitElem = kitElem->GetNextElement("kit");
        continue;
      }

      ariac::Kit kit;

      // Parse the tray the kit is assigned to.
      std::string trayID;
      if (kitElem->HasElement("tray"))
      {
        trayID = kitElem->Get<std::string>("tray");
      }

      // Parse the objects inside the kit.
      sdf::ElementPtr objectElem = kitElem->GetElement("object");
      while (objectElem)
      {
        // Parse the object type.
        if (!objectElem->HasElement("type"))
        {
          gzerr << "Unable to find <type> in object.\n";
          objectElem = objectElem->GetNextElement("object");
          continue;
        }
        sdf::ElementPtr typeElement = objectElem->GetElement("type");
        std::string type = typeElement->Get<std::string>();

        // Parse the object pose (optional).
        if (!objectElem->HasElement("pose"))
        {
          gzerr << "Unable to find <pose> in object.\n";
          objectElem = objectElem->GetNextElement("object");
          continue;
        }
        sdf::ElementPtr poseElement = objectElem->GetElement("pose");
        math::Pose pose = poseElement->Get<math::Pose>();

        // Add the object to the kit.
        ariac::KitObject obj = {type, pose};
        kit.objects.push_back(obj);

        objectElem = objectElem->GetNextElement("object");
      }

      // Add a new kit to the collection of kits.
      kits[trayID] = kit;

      kitElem = kitElem->GetNextElement("kit");
    }

    // Add a new goal.
    ariac::GoalID_t goalID = "goal_" + std::to_string(goalCount++);
    ariac::Goal goal = {goalID, startTime, allowedTime, kits, 0.0};
    this->dataPtr->goalsToAnnounce.push_back(goal);

    goalElem = goalElem->GetNextElement("goal");
  }

  // Sort the goals by their start times.
  std::sort(this->dataPtr->goalsToAnnounce.begin(), this->dataPtr->goalsToAnnounce.end());

  // Debug output.
  // gzdbg << "Goals:" << std::endl;
  // for (auto goal : this->dataPtr->goalsToAnnounce)
  //   gzdbg << goal << std::endl;

  // Initialize ROS
  this->dataPtr->rosnode.reset(new ros::NodeHandle(robotNamespace));

  // Publisher for announcing new goals.
  this->dataPtr->goalPub = this->dataPtr->rosnode->advertise<
    osrf_gear::Goal>(goalsTopic, 1000, true);  // latched=true

  // Publisher for announcing new state of Gazebo's task.
  this->dataPtr->gazeboTaskStatePub = this->dataPtr->rosnode->advertise<
    std_msgs::String>(gazeboTaskStateTopic, 1000);

  // Subscribe to the topic for receiving the state of the team's task.
  this->dataPtr->teamStartServiceServer =
    this->dataPtr->rosnode->advertiseService(teamStartServiceName,
      &ROSAriacTaskManagerPlugin::HandleStartService, this);

  // Initialize Gazebo transport.
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->populatePub =
    this->dataPtr->node->Advertise<msgs::GzString>(conveyorActivationTopic);

  // Initialize the game scorer.
  this->dataPtr->trayInfoSub = this->dataPtr->rosnode->subscribe(
    "/ariac/trays", 10, &AriacScorer::OnTrayInfoReceived, &this->dataPtr->ariacScorer);

  this->dataPtr->gameStartTime = this->dataPtr->world->GetSimTime();

  this->dataPtr->connection = event::Events::ConnectWorldUpdateEnd(
    boost::bind(&ROSAriacTaskManagerPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto currentSimTime = this->dataPtr->world->GetSimTime();

  double elapsedTime = (currentSimTime - this->dataPtr->lastUpdateTime).Double();
  if (this->dataPtr->currentState == "ready")
  {
    this->dataPtr->gameStartTime = currentSimTime;
    this->dataPtr->currentState = "go";

    this->PopulateConveyorBelt();
  }
  else if (this->dataPtr->currentState == "go")
  {
    // Update the goal manager.
    this->ProcessGoalsToAnnounce();

    // Update the score.
    this->dataPtr->ariacScorer.Update();
    auto gameScore = this->dataPtr->ariacScorer.GetGameScore();
    if (gameScore.total() != this->dataPtr->currentGameScore.total())
    {
      ROS_INFO_STREAM("Current game score: " << gameScore.total());
      this->dataPtr->currentGameScore = gameScore;
    }

    if(!this->dataPtr->goalsInProgress.empty())
    {
      this->dataPtr->goalsInProgress.top().timeTaken += elapsedTime;
      auto goalID = this->dataPtr->goalsInProgress.top().goalID;
      auto timeTaken = this->dataPtr->goalsInProgress.top().timeTaken;

      // Check for completed goals.
      bool goalCompleted = this->dataPtr->ariacScorer.IsCurrentGoalComplete();
      if (goalCompleted)
      {
        ROS_INFO_STREAM("Goal complete: " << goalID);
        this->StopCurrentGoal();
      }
      else
      {
        // Check if the time limit for the current goal has been exceeded.
        bool goalTimedOut = timeTaken > this->dataPtr->goalsInProgress.top().allowedTime;
        if (goalTimedOut)
        {
          ROS_INFO_STREAM("Goal timed out: " << goalID);
          this->StopCurrentGoal();
        }
      }
    }
  }

  // ToDo: Publish at a lower frequency.
  std_msgs::String msg;
  msg.data = this->dataPtr->currentState;
  this->dataPtr->gazeboTaskStatePub.publish(msg);
  this->dataPtr->lastUpdateTime = currentSimTime;
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::ProcessGoalsToAnnounce()
{
  if (this->dataPtr->goalsToAnnounce.empty())
    return;

  // Check whether announce a new goal from the list.
  auto elapsed = this->dataPtr->world->GetSimTime() - this->dataPtr->gameStartTime;
  if (elapsed.Double() >= this->dataPtr->goalsToAnnounce.front().startTime)
  {
    auto goal = this->dataPtr->goalsToAnnounce.front();

    // Move goal to the 'in process' stack
    this->dataPtr->goalsInProgress.push(ariac::Goal(goal));
    this->dataPtr->goalsToAnnounce.erase(this->dataPtr->goalsToAnnounce.begin());

    this->AssignGoal(goal);
  }
}

/////////////////////////////////////////////////
bool ROSAriacTaskManagerPlugin::HandleStartService(
  std_srvs::Trigger::Request & req,
  std_srvs::Trigger::Response & res)
{
  (void)req;
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (this->dataPtr->currentState == "init") {
    this->dataPtr->currentState = "ready";
    res.success = true;
    res.message = "competition started successfully!";
    return true;
  }
  res.success = false;
  res.message = "cannot start if not in 'init' state";
  return true;
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::PopulateConveyorBelt()
{
  // Publish a message on the activation_plugin of the PopulationPlugin.
  gazebo::msgs::GzString msg;
  msg.set_data("restart");
  this->dataPtr->populatePub->Publish(msg);
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::AssignGoal(const ariac::Goal & goal)
{
    osrf_gear::Goal goalMsg;
    fillGoalMsg(goal, goalMsg);
    osrf_gear::GoalConstPtr goalMsgConstPtr(new osrf_gear::Goal(goalMsg));

    // Publish the goal to ROS topic
    ROS_INFO_STREAM("Announcing goal: " << goal.goalID);
    this->dataPtr->goalPub.publish(goalMsg);

    // Assign the scorer the goal to monitor
    gzdbg << "Assigning goal: " << goal << std::endl;
    // TODO: don't pass this a ROS message unnecessarily
    this->dataPtr->ariacScorer.OnGoalReceived(goalMsgConstPtr);
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::StopCurrentGoal()
{
  if (this->dataPtr->goalsInProgress.size())
  {
    ROS_INFO_STREAM("Stopping goal: " << this->dataPtr->goalsInProgress.top().goalID);
    this->dataPtr->goalsInProgress.pop();
    this->dataPtr->ariacScorer.UnassignCurrentGoal();
  }

  if (this->dataPtr->goalsInProgress.size())
  {
    // Assign the previous goal to the scorer
    auto goal = this->dataPtr->goalsInProgress.top();
    ROS_INFO_STREAM("Restoring goal: " << goal.goalID);
    this->AssignGoal(goal);
  }
}
