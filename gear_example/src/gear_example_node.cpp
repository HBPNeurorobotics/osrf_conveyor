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

#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <osrf_gear/Goal.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
}

/// Example class that can hold state and provide methods that handle incoming data.
class MyCompetitionClass
{
public:
  explicit MyCompetitionClass(ros::NodeHandle & node) : has_been_zeroed_(false) {
    joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/arm/command", 10);
  }

  /// Called when a new message is received on the '/ariac/current_score' topic.
  void current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
    if (msg->data != current_score_)
    {
      ROS_INFO_STREAM("Score: " << msg->data);
    }
    current_score_ = msg->data;
  }

  /// Called when a new message is received on the '/ariac/competition_state' topic.
  void competition_state_callback(const std_msgs::String::ConstPtr & msg) {
    if (msg->data == "done" && competition_state_ != "done")
    {
      ROS_INFO("Competition ended.");
    }
    competition_state_ = msg->data;
  }
  /// Called when a new Goal message is received on the '/ariac/orders' topic.
  void goal_callback(const osrf_gear::Goal::ConstPtr & goal_msg) {
    ROS_INFO_STREAM("Received goal:\n" << *goal_msg);
    received_goals_.push_back(*goal_msg);
  }

  /// Called when a new JointState message is received on the '/ariac/arm/joint_states' topic.
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

  /// Create a JointTrajectory with all positions set to zero, and command the arm.
  void send_arm_to_zero_state() {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;
    // Copy the joint names from the data received on the '/ariac/arm/joint_states' topic.
    msg.joint_names = current_joint_states_.name;
    // Create one point in the trajectory.
    msg.points.resize(1);
    // Resize the vector to the same length as the joint names, with all positions at 0.0.
    msg.points[0].positions.resize(current_joint_states_.name.size(), 0.0);
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(1.0);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher_.publish(msg);
  }

  /// Called when a new LogicalCameraImage message is received on the '/ariac/logical_camera' topic.
  void logical_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    ROS_INFO_STREAM_THROTTLE(10, "Logical camera sees '" << image_msg->models.size() << "' objects.");
  }

  /// Called when a new Proximity message is received on the '/ariac/break_beam_changed' topic.
  void break_beam_callback(const osrf_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {  // If there is an object in proximity.
      ROS_INFO("Break beam triggered.");
    }
  }

private:
  std::string competition_state_;
  double current_score_;
  ros::Publisher joint_trajectory_publisher_;
  std::vector<osrf_gear::Goal> received_goals_;
  sensor_msgs::JointState current_joint_states_;
  bool has_been_zeroed_;
};

void proximity_sensor_callback(const osrf_gear::Proximity::ConstPtr & msg) {
  if (msg->object_detected) {  // If there is an object in proximity.
    ROS_INFO("Proximity sensor triggered.");
  }
}

void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
  size_t number_of_valid_ranges = std::count_if(msg->ranges.begin(), msg->ranges.end(), std::isfinite<float>);
  if (number_of_valid_ranges > 0) {
    ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
  }
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "gear_example_node");  // Last argument is the default name of the node.

  ros::NodeHandle node;

  MyCompetitionClass comp_class(node);  // Instance of custom class from above.

  // "Connect" new data on the '/ariac/current_score' topic to the callback in the custom class.
  ros::Subscriber current_score_subscriber = node.subscribe(
    "/ariac/current_score", 10, &MyCompetitionClass::current_score_callback, &comp_class);

  // "Connect" new data on the '/ariac/competition_state' topic to callback in the custom class.
  ros::Subscriber competition_state_subscriber = node.subscribe(
    "/ariac/competition_state", 10, &MyCompetitionClass::competition_state_callback, &comp_class);

  // "Connect" new data on the '/ariac/orders' topic to the callback in the custom class.
  ros::Subscriber goals_subscriber = node.subscribe(
    "/ariac/orders", 10, &MyCompetitionClass::goal_callback, &comp_class);

  // "Connect" new data on the '/ariac/arm/joint_states' topic to the callback in the custom class.
  ros::Subscriber joint_state_subscriber = node.subscribe(
    "/ariac/arm/joint_states", 10, &MyCompetitionClass::joint_state_callback, &comp_class);

  // "Connect" new data on the '/ariac/proximity_sensor_changed' topic to the free-function callback.
  ros::Subscriber proximity_sensor_subscriber = node.subscribe(
    "/ariac/proximity_sensor_changed", 10, proximity_sensor_callback);

  // "Connect" new data on the '/ariac/break_beam_changed' topic to the callback in the custom class.
  ros::Subscriber break_beam_subscriber = node.subscribe(
    "/ariac/break_beam_changed", 10, &MyCompetitionClass::break_beam_callback, &comp_class);

  // "Connect" new data on the '/ariac/logical_camera' topic to the callback in the custom class.
  ros::Subscriber logical_camera_subscriber = node.subscribe(
    "/ariac/logical_camera", 10, &MyCompetitionClass::logical_camera_callback, &comp_class);

  // "Connect" new data on the '/ariac/laser_profiler' topic to the free-function callback.
  ros::Subscriber laser_profiler_subscriber = node.subscribe(
    "/ariac/laser_profiler", 10, laser_profiler_callback);

  ROS_INFO("Setup complete.");
  start_competition(node);
  ros::spin();  // This executes callbacks on new data until ctrl-c.

  return 0;
}
