/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_ACTION_HPP_
#define HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_ACTION_HPP_

#include <limits>
#include <memory>
#include <string>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_server_goal_handle.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <hsrb_gripper_controller/hrh_gripper_controller.hpp>

namespace hsrb_gripper_controller {

// Default action monitor cycle [Hz]
constexpr double kDefaultActionMonitorRate = 20.0;

// Parameter retrieval with default value
template <typename ParameterType>
auto GetParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& name,
                  const ParameterType& default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<ParameterType>(name, default_value);
  } else {
    return node->get_parameter(name).get_value<ParameterType>();
  }
}

// Parameter retrieval with default value, using default value even if the parameter is invalid
template <typename ParameterType>
auto GetPositiveParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& name,
                          const ParameterType& default_value) {
  auto value = GetParameter(node, name, default_value);
  if (value < std::numeric_limits<double>::min()) {
    RCLCPP_WARN_STREAM(node->get_logger(), name << " must be positive. Use default value " << default_value);
    value = default_value;
  }
  return value;
}

// Parameter retrieval with default value, using default value even if the parameter is negative
template <typename ParameterType>
auto GetNonNegativeParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& name,
                             const ParameterType& default_value) {
  auto value = GetParameter(node, name, default_value);
  if (value < 0.0) {
    RCLCPP_WARN_STREAM(node->get_logger(), name << " must not be negative. Use default value " << default_value);
    value = default_value;
  }
  return value;
}

bool ValidateTrajectory(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                        const trajectory_msgs::msg::JointTrajectory& trajectory,
                        const std::string& joint_name);

/// @struct GoalCondition
/// @brief Goal condition
struct GoalCondition {
  /// Command position
  double goal;
  /// Target arrival time
  rclcpp::Time expected_arrival_time;
  /// Time to stop trajectory tracking
  rclcpp::Time abort_time;
  /// Allowable error for goal position
  double goal_tolerance;
};

/// @class IHrhGripperAction
/// @brief Gripper action interface class
class IHrhGripperAction : public std::enable_shared_from_this<IHrhGripperAction> {
 public:
  using Ptr = std::shared_ptr<IHrhGripperAction>;

  virtual ~IHrhGripperAction() = default;

  /// Returns the control mode used by the action
  virtual int32_t target_mode() const = 0;

  /// Initialization
  /// @param [in] node SharedPtr of the node
  /// @return true: success false: failure
  virtual bool Init(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) = 0;

  /// Activation
  /// @param [in] node SharedPtr of the node
  /// @return true: success false: failure
  virtual bool Activate() = 0;

  /// Periodic update process
  /// @param [in] time Current time
  virtual void Update(const rclcpp::Time& time) = 0;

  /// Interrupt the active goal
  virtual void PreemptActiveGoal() = 0;

  /// Retrieve command value
  /// @return Command value
  virtual trajectory_msgs::msg::JointTrajectoryPoint GetReferenceState() = 0;

  /// Retrieve current value
  /// @return Current value
  virtual trajectory_msgs::msg::JointTrajectoryPoint GetFeedbackState() = 0;
};


/// @class HrhGripperAction
/// @brief Gripper action class
template <class ActionType>
class HrhGripperAction : public IHrhGripperAction {
 public:
  /// Constructor
  /// @param [in] controller Controller
  /// @param [in] action_name Name of the provided action
  /// @param [in] target_mode Control mode used by the action
  HrhGripperAction(HrhGripperController* controller, const std::string& action_name, int32_t target_mode)
      : controller_(controller), action_name_(action_name), target_mode_(target_mode) {}
  virtual ~HrhGripperAction() = default;

  /// Returns the control mode used by the action
  int32_t target_mode() const override { return target_mode_; }

  /// Initialization
  /// @param [in] node SharedPtr of the node
  /// @return true: success false: failure
  bool Init(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) override {
    node_ = node;

    double action_monitor_rate = GetPositiveParameter(node, "action_monitor_rate", kDefaultActionMonitorRate);
    action_monitor_period_ = 1.0 / action_monitor_rate;
    std::string gripper_namespace = GetParameter(node, "namespace", "~");
    std::string interface_name = gripper_namespace + action_name_;

    if (!InitImpl(node)) {
      return false;
    }
    action_server_ = rclcpp_action::create_server<ActionType>(
        node, interface_name,
        std::bind(&HrhGripperAction<ActionType>::GoalCallback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&HrhGripperAction<ActionType>::CancelCallback, this, std::placeholders::_1),
        std::bind(&HrhGripperAction<ActionType>::FeedbackSetupCallback, this, std::placeholders::_1));
    return true;
  }

  /// Activation
  /// @param [in] node SharedPtr of the node
  /// @return true: success false: failure
  bool Activate() override { return true; }

  /// Interrupt the active goal
  void PreemptActiveGoal() override {
    auto active_goal = *goal_handle_buffer_.readFromNonRT();
    active_goal.reset();
    goal_handle_timer_.reset();
    goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
  }

  /// Check if the action is active
  /// @return true: active false: inactive
  bool IsActive() const {
    const auto active_goal = *goal_handle_buffer_.readFromNonRT();
    if (!active_goal || !active_goal->valid()) {
      return false;
    }
    return true;
  }

 protected:
  // Action's GoalHandle
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<ActionType>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;

  realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr> goal_handle_buffer_;

  /// Action server
  typename rclcpp_action::Server<ActionType>::SharedPtr action_server_;

  /// Process when action is accepted
  rclcpp_action::GoalResponse GoalCallback(const rclcpp_action::GoalUUID& uuid,
                                           std::shared_ptr<const typename ActionType::Goal> goal) {
    if (!controller_->IsAcceptable()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (ValidateGoal(*goal)) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  /// Process when action execution starts
  void FeedbackSetupCallback(std::shared_ptr<GoalHandle> goal_handle) {
    controller_->PreemptActiveGoal();
    controller_->ChangeControlMode(shared_from_this());
    UpdateActionImpl(*goal_handle->get_goal());

    auto realtime_goal_handle = std::make_shared<RealtimeGoalHandle>(goal_handle);
    realtime_goal_handle->execute();
    goal_handle_buffer_.writeFromNonRT(realtime_goal_handle);

    goal_handle_timer_ = node_->create_wall_timer(std::chrono::duration<double>(action_monitor_period_),
                                                  std::bind(&RealtimeGoalHandle::runNonRealtime, realtime_goal_handle));
  }

  /// Process when action is canceled
  /// @param [in] goal_handle Goal handle
  rclcpp_action::CancelResponse CancelCallback(const std::shared_ptr<GoalHandle> goal_handle) {
    const auto active_goal = *goal_handle_buffer_.readFromNonRT();
    if (active_goal && active_goal->gh_ == goal_handle) {
      PreemptActiveGoal();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /// Implementation of action initialization
  virtual bool InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) { return true; }
  /// Check if the goal is acceptable
  virtual bool ValidateGoal(const typename ActionType::Goal& goal) { return true; }
  /// Update the action's target
  virtual void UpdateActionImpl(const typename ActionType::Goal& goal) = 0;

  /// Controller
  // std::shared_ptr<HrhGripperController> controller_;
  HrhGripperController* controller_;
  /// Action name
  std::string action_name_;
  /// Control mode targeted by this action
  int32_t target_mode_;
  /// Action state update cycle
  double action_monitor_period_;

  /// SharedPtr of the node
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  /// Timer for action execution
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
};

}  // namespace hsrb_gripper_controller

#endif  // HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_ACTION_HPP_
