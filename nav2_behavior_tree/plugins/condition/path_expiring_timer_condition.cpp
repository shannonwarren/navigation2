// Copyright (c) 2022 Joshua Wallace
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

#include <ctime>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"

#include "nav2_behavior_tree/plugins/condition/path_expiring_timer_condition.hpp"

namespace nav2_behavior_tree {

PathExpiringTimerCondition::PathExpiringTimerCondition(
    const std::string &condition_name, const BT::NodeConfiguration &conf)
    : BT::ConditionNode(condition_name, conf), period_(1.0), first_time_(true),
      global_frame_("map"), robot_base_frame_("base_link") {
  getInput("seconds", period_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

BT::NodeStatus PathExpiringTimerCondition::tick() {
  if (first_time_) {
    getInput("path", prev_path_);
    first_time_ = false;
    start_ = node_->now();
    return BT::NodeStatus::FAILURE;
  }

  // if (isRobotNearGoal()) {
  //   return BT::NodeStatus::FAILURE;
  // }

  // Grab the new path
  nav_msgs::msg::Path path;
  getInput("path", path);

  if (prev_path_ != path) {
    prev_path_ = path;
    start_ = node_->now();
  }

  // Determine how long its been since we've started this iteration
  auto elapsed = node_->now() - start_;

  // Now, get that in seconds
  auto seconds = elapsed.seconds();

  if (seconds < period_) {
    return BT::NodeStatus::FAILURE;
  }

  start_ = node_->now(); // Reset the timer
  return BT::NodeStatus::SUCCESS;
}

bool PathExpiringTimerCondition::isRobotNearGoal() {
  geometry_msgs::msg::PoseStamped current_pose;

  if (!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_,
                                 robot_base_frame_, transform_tolerance_)) {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }

  geometry_msgs::msg::PoseStamped goal;
  getInput("goal", goal);
  double dx = goal.pose.position.x - current_pose.pose.position.x;
  double dy = goal.pose.position.y - current_pose.pose.position.y;
  std::cout << dx * dx + dy * dy << std::endl;
  return (dx * dx + dy * dy) < (0.15);
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::PathExpiringTimerCondition>(
      "PathExpiringTimer");
}
