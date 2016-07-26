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

#include <vector>

#include <ros/ros.h>

#include <osrf_gear/Goal.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

void start_competition(ros::NodeHandle & node) {
  ros::ServiceClient start_client =
    node.serviceClient<std_srvs::Trigger>("/ariac_task_manager/start");
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;
  start_client.call(srv);
  if (!srv.response.success) {
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
}

class MyCompetitionClass {
public:
  explicit MyCompetitionClass(ros::NodeHandle & node) : has_been_zeroed_(false) {
    joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/arm_controller/command", 10);
  }

  void goal_callback(const osrf_gear::Goal::ConstPtr & goal_msg) {
    ROS_INFO_STREAM("Received goal:\n" << *goal_msg);
    received_goals_.push_back(*goal_msg);
  }

  void joint_state_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg) {
    ROS_INFO_STREAM_THROTTLE(10, "Current Joint States (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // ROS_INFO_STREAM("Current Joint States:\n" << *joint_state_msg);
    current_joint_states_ = *joint_state_msg;
    if (!has_been_zeroed_) {
      has_been_zeroed_ = true;
      ROS_INFO("Sending arm to zero joint positions...");
      send_arm_to_zero_state();
    }
  }

  void send_arm_to_zero_state() {
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names = current_joint_states_.name;
    msg.points.resize(1);
    msg.points[0].positions.resize(current_joint_states_.name.size(), 0.0);
    msg.points[0].time_from_start = ros::Duration(1.0);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher_.publish(msg);
  }

private:
  ros::Publisher joint_trajectory_publisher_;
  std::vector<osrf_gear::Goal> received_goals_;
  sensor_msgs::JointState current_joint_states_;
  bool has_been_zeroed_;
};

int main(int argc, char ** argv) {
  ros::init(argc, argv, "gear_example_node");

  ros::NodeHandle node;

  MyCompetitionClass comp_class(node);
  ros::Subscriber goals_subscriber = node.subscribe(
    "/ariac_task_manager/goals", 10, &MyCompetitionClass::goal_callback, &comp_class);
  ros::Subscriber joint_state_subscriber = node.subscribe(
    "/joint_states", 10, &MyCompetitionClass::joint_state_callback, &comp_class);

  ROS_INFO("Setup complete.");
  start_competition(node);
  ros::spin();

  return 0;
}
