// Copyright (c) 2021 Joshua Wallace
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

#include "nav2_behavior_tree/plugins/condition/is_path_valid_condition.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace nav2_behavior_tree {

IsPathValidCondition::IsPathValidCondition(const std::string &condition_name,
                                           const BT::NodeConfiguration &conf)
    : BT::ConditionNode(condition_name, conf) {
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<nav2_msgs::srv::IsPathValid>("is_path_valid");

  server_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>(
          "server_timeout");
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
}

BT::NodeStatus IsPathValidCondition::tick() {
  nav_msgs::msg::Path path;
  getInput("path", path);

  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();

  request->path = path;
  auto result = client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, result, server_timeout_) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    if (result.get()->is_valid) {
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::FAILURE;
}

// bool IsPathValidCondition::isGoalReached()
// {
//   geometry_msgs::msg::PoseStamped current_pose;

//   if (!nav2_util::getCurrentPose(
//       current_pose, *tf_, global_frame_, robot_base_frame_,
//       transform_tolerance_))
//   {
//     RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not
//     available."); return false;
//   }

//   geometry_msgs::msg::PoseStamped goal;
//   getInput("goal", goal);
//   double dx = goal.pose.position.x - current_pose.pose.position.x;
//   double dy = goal.pose.position.y - current_pose.pose.position.y;

//   return (dx * dx + dy * dy) <= (goal_reached_tol_ * goal_reached_tol_);
// }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::IsPathValidCondition>(
      "IsPathValid");
}
