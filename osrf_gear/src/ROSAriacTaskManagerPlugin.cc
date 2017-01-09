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
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>


#include "osrf_gear/ARIAC.hh"
#include "osrf_gear/ROSAriacTaskManagerPlugin.hh"
#include "osrf_gear/AriacScorer.h"
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/ConveyorBeltState.h>
#include "osrf_gear/Goal.h"
#include "osrf_gear/Kit.h"
#include "osrf_gear/KitObject.h"
#include "osrf_gear/VacuumGripperState.h"

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

    /// \brief ROS subscriber for the gripper state.
    public: ros::Subscriber gripperStateSub;

    /// \brief Publishes the Gazebo task state.
    public: ros::Publisher taskStatePub;

    /// \brief Publishes the game score total.
    public: ros::Publisher taskScorePub;

    /// \brief Service that allows the user to start the competition.
    public: ros::ServiceServer teamStartServiceServer;

    /// \brief Service that allows a tray to be submitted for inspection.
    public: ros::ServiceServer submitTrayServiceServer;

    /// \brief Transportation node.
    public: transport::NodePtr node;

    /// \brief Publisher for enabling the object population on the conveyor.
    public: transport::PublisherPtr populatePub;

    /// \brief Client for controlling the conveyor.
    public: ros::ServiceClient conveyorControlClient;

    /// \brief Timer for regularly publishing state/score.
    public: ros::Timer statusPubTimer;

    /// \brief Connection event.
    public: event::ConnectionPtr connection;

    /// \brief The time the last update was called.
    public: common::Time lastUpdateTime;

    /// \brief The time specified in the object is relative to this time.
    public: common::Time gameStartTime;

    /// \brief The time in seconds that has been spent on the current goal.
    public: double timeSpentOnCurrentGoal;

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
    msgKit.kit_type.data = item.first;
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

  std::string taskStateTopic = "competition_state";
  if (_sdf->HasElement("task_state_topic"))
    taskStateTopic = _sdf->Get<std::string>("task_state_topic");

  std::string taskScoreTopic = "current_score";
  if (_sdf->HasElement("task_score_topic"))
    taskScoreTopic = _sdf->Get<std::string>("task_score_topic");

  std::string conveyorControlTopic = "conveyor/control";
  if (_sdf->HasElement("conveyor_control_topic"))
    conveyorControlTopic = _sdf->Get<std::string>("conveyor_control_topic");

  std::string populationActivateTopic = "populate_belt";
  if (_sdf->HasElement("population_activate_topic"))
    populationActivateTopic = _sdf->Get<std::string>("population_activate_topic");

  std::string goalsTopic = "orders";
  if (_sdf->HasElement("goals_topic"))
    goalsTopic = _sdf->Get<std::string>("goals_topic");

  std::string submitTrayServiceName = "submit_tray";
  if (_sdf->HasElement("submit_tray_service_name"))
    submitTrayServiceName = _sdf->Get<std::string>("submit_tray_service_name");


  // Parse the goals.
  sdf::ElementPtr goalElem = NULL;
  if (_sdf->HasElement("goal"))
  {
    goalElem = _sdf->GetElement("goal");
  }

  unsigned int goalCount = 0;
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

      // Parse the kit type.
      ariac::KitType_t kitType;
      if (kitElem->HasElement("kit_type"))
      {
        kitType = kitElem->Get<std::string>("kit_type");
      }
      kit.kitType = kitType;

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
      kits[kitType] = kit;

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

  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Publisher for announcing new goals.
  this->dataPtr->goalPub = this->dataPtr->rosnode->advertise<
    osrf_gear::Goal>(goalsTopic, 1000, true);  // latched=true

  // Publisher for announcing new state of the competition.
  this->dataPtr->taskStatePub = this->dataPtr->rosnode->advertise<
    std_msgs::String>(taskStateTopic, 1000);

  // Publisher for announcing the score of the game.
  this->dataPtr->taskScorePub = this->dataPtr->rosnode->advertise<
    std_msgs::Float32>(taskScoreTopic, 1000);

  // Service for starting the competition.
  this->dataPtr->teamStartServiceServer =
    this->dataPtr->rosnode->advertiseService(teamStartServiceName,
      &ROSAriacTaskManagerPlugin::HandleStartService, this);

  // Service for submitting trays for inspection.
  this->dataPtr->submitTrayServiceServer =
    this->dataPtr->rosnode->advertiseService(submitTrayServiceName,
      &ROSAriacTaskManagerPlugin::HandleSubmitTrayService, this);

  // Client for the conveyor control commands.
  this->dataPtr->conveyorControlClient =
    this->dataPtr->rosnode->serviceClient<osrf_gear::ConveyorBeltControl>(
      conveyorControlTopic);

  // Timer for regularly publishing state/score.
  this->dataPtr->statusPubTimer =
    this->dataPtr->rosnode->createTimer(ros::Duration(0.1),
      &ROSAriacTaskManagerPlugin::PublishStatus, this);

  // Initialize Gazebo transport.
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->populatePub =
    this->dataPtr->node->Advertise<msgs::GzString>(populationActivateTopic);

  // Initialize the game scorer.
  this->dataPtr->trayInfoSub = this->dataPtr->rosnode->subscribe(
    "/ariac/trays", 10, &AriacScorer::OnTrayInfoReceived, &this->dataPtr->ariacScorer);
  this->dataPtr->gripperStateSub = this->dataPtr->rosnode->subscribe(
    "/ariac/gripper/state", 10, &AriacScorer::OnGripperStateReceived,
    &this->dataPtr->ariacScorer);

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

    this->ControlConveyorBelt(0.2);
    this->PopulateConveyorBelt();
  }
  else if (this->dataPtr->currentState == "go")
  {
    // Update the goal manager.
    this->ProcessGoalsToAnnounce();

    // Update the score.
    this->dataPtr->ariacScorer.Update(elapsedTime);
    auto gameScore = this->dataPtr->ariacScorer.GetGameScore();
    if (gameScore.total() != this->dataPtr->currentGameScore.total())
    {
      std::ostringstream logMessage;
      logMessage << "Current game score: " << gameScore.total();
      ROS_INFO_STREAM(logMessage.str().c_str());
      gzdbg << logMessage.str() << std::endl;
      this->dataPtr->currentGameScore = gameScore;
    }

    if (!this->dataPtr->goalsInProgress.empty())
    {
      this->dataPtr->goalsInProgress.top().timeTaken += elapsedTime;
      auto goalID = this->dataPtr->goalsInProgress.top().goalID;
      // TODO: timing should probably be managed by the scorer but we want to use sim time
      this->dataPtr->timeSpentOnCurrentGoal = this->dataPtr->goalsInProgress.top().timeTaken;

      // Check for completed goals.
      bool goalCompleted = this->dataPtr->ariacScorer.IsCurrentGoalComplete();
      if (goalCompleted)
      {
        std::ostringstream logMessage;
        logMessage << "Order complete: " << goalID;
        ROS_INFO_STREAM(logMessage.str().c_str());
        gzdbg << logMessage.str() << std::endl;
        this->StopCurrentGoal();
      }
      else
      {
        // Check if the time limit for the current goal has been exceeded.
        if (this->dataPtr->timeSpentOnCurrentGoal > this->dataPtr->goalsInProgress.top().allowedTime)
        {
          std::ostringstream logMessage;
          logMessage << "Order timed out: " << goalID;
          ROS_INFO_STREAM(logMessage.str().c_str());
          gzdbg << logMessage.str() << std::endl;
          this->StopCurrentGoal();
        }
      }
    }

    if (this->dataPtr->goalsInProgress.empty() && this->dataPtr->goalsToAnnounce.empty())
    {
      this->dataPtr->currentGameScore.totalProcessTime =
        (currentSimTime - this->dataPtr->gameStartTime).Double();
      this->dataPtr->currentState = "end_game";
    }
  }
  else if (this->dataPtr->currentState == "end_game")
  {
    std::ostringstream logMessage;
    logMessage << "No more orders to process. Final score: " << \
      this->dataPtr->currentGameScore.total() << "\nScore breakdown:\n" << \
      this->dataPtr->currentGameScore;
    ROS_INFO_STREAM(logMessage.str().c_str());
    gzdbg << logMessage.str() << std::endl;
    this->dataPtr->currentState = "done";
  }

  this->dataPtr->lastUpdateTime = currentSimTime;
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::PublishStatus(const ros::TimerEvent&)
{
  std_msgs::Float32 scoreMsg;
  scoreMsg.data = this->dataPtr->currentGameScore.total();
  this->dataPtr->taskScorePub.publish(scoreMsg);

  std_msgs::String stateMsg;
  stateMsg.data = this->dataPtr->currentState;
  this->dataPtr->taskStatePub.publish(stateMsg);
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
    gzdbg << "New goal to announce: " << goal.goalID << std::endl;

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
bool ROSAriacTaskManagerPlugin::HandleSubmitTrayService(
  osrf_gear::SubmitTray::Request & req,
  osrf_gear::SubmitTray::Response & res)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (this->dataPtr->currentState != "go") {
    std::string errStr = "Competition is not running so trays cannot be submitted.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    return false;
  }

  ariac::KitTray kitTray;
  gzdbg << "SubmitTray request received for tray: " << req.tray_id.data << std::endl;
  if (!this->dataPtr->ariacScorer.GetTrayById(req.tray_id.data, kitTray))
  {
    res.success = false;
    return true;
  }
  kitTray.currentKit.kitType = req.kit_type.data;
  res.success = true;
  res.inspection_result = this->dataPtr->ariacScorer.SubmitTray(kitTray).total();
  gzdbg << "Inspection result: " << res.inspection_result << std::endl;
  return true;
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::ControlConveyorBelt(double velocity)
{

  if (!this->dataPtr->conveyorControlClient.exists())
  {
    this->dataPtr->conveyorControlClient.waitForExistence();
  }

  // Make a service call to set the velocity of the belt
  osrf_gear::ConveyorBeltState controlMsg;
  controlMsg.velocity = velocity;
  osrf_gear::ConveyorBeltControl srv;
  srv.request.state = controlMsg;
  this->dataPtr->conveyorControlClient.call(srv);
  if (!srv.response.success) {
    std::string errStr = "Failed to control conveyor.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
  }
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
    // Publish the goal to ROS topic
    gzdbg << "Announcing order: " << goal.goalID << std::endl;
    osrf_gear::Goal goalMsg;
    fillGoalMsg(goal, goalMsg);
    this->dataPtr->goalPub.publish(goalMsg);

    // Assign the scorer the goal to monitor
    gzdbg << "Assigning order: " << goal << std::endl;
    this->dataPtr->ariacScorer.AssignGoal(goal);
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::StopCurrentGoal()
{
  if (this->dataPtr->goalsInProgress.size())
  {
    gzdbg << "Stopping order: " << this->dataPtr->goalsInProgress.top().goalID << std::endl;
    this->dataPtr->goalsInProgress.pop();
    this->dataPtr->ariacScorer.UnassignCurrentGoal(this->dataPtr->timeSpentOnCurrentGoal);
  }

  if (this->dataPtr->goalsInProgress.size())
  {
    // Assign the previous goal to the scorer
    auto goal = this->dataPtr->goalsInProgress.top();
    gzdbg << "Restoring order: " << goal.goalID << std::endl;
    this->AssignGoal(goal);
  }
}
