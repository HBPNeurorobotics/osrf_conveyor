// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ros/ros.h>

#include "osrf_gear/ARIAC.hh"
#include "osrf_gear/AriacScorer.h"

int main(int argc, char ** argv) {
  ros::init(argc, argv, "ariac_scorer_node");

  ros::NodeHandle node;
  AriacScorer ariac_scorer(node);

  ros::Subscriber goal_subscriber = node.subscribe(
    "/ariac/goals", 10, &AriacScorer::OnGoalReceived, &ariac_scorer);
  ros::Subscriber tray_subscriber = node.subscribe(
    "/ariac/trays", 10, &AriacScorer::OnTrayInfoReceived, &ariac_scorer);

  ros::Rate rate(1);
  while(ros::ok())
  {
    ariac_scorer.Update();
    ariac::GameScore gameScore = ariac_scorer.GetGameScore();
    ROS_INFO_STREAM("Current game score: " << gameScore.total());
    bool currentGoalComplete = ariac_scorer.IsCurrentGoalComplete();
    ROS_INFO_STREAM("Current goal is complete: " << currentGoalComplete);
    ros::spinOnce();
    rate.sleep();
  }
}
