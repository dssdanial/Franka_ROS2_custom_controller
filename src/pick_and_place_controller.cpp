// Copyright (c) 2023 Franka Robotics GmbH
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

#include <franka_example_controllers/pick_and_place_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
PickAndPlaceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
PickAndPlaceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }

  return config;
}

controller_interface::return_type PickAndPlaceController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {
  
  if (initialization_flag_) {
    for (int i = 0; i < num_joints; ++i) {
      initial_q_.at(i) = state_interfaces_[i].get_value();
      phase_start_position_.at(i) = initial_q_.at(i);
    }
    initialization_flag_ = false;
    elapsed_time_ = 0.0;
    phase_start_time_ = 0.0;
    current_phase_ = Phase::MOVE_TO_PICK;
    
    RCLCPP_INFO(rclcpp::get_logger("PickAndPlaceController"), 
                "Pick and place controller initialized. Starting sequence...");
  } else {
    elapsed_time_ += period.seconds();
  }

  updatePhase();
  setTargetPositions();

  // Calculate trajectory for current phase
  double phase_elapsed = elapsed_time_ - phase_start_time_;
  std::array<double, 7> target_position;
  
  calculateTrajectory(phase_start_position_, current_target_, 
                     phase_elapsed, phase_duration_, target_position);

  // Set command interfaces
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(target_position[i]);
  }

  return controller_interface::return_type::OK;
}

void PickAndPlaceController::updatePhase() {
  double phase_elapsed = elapsed_time_ - phase_start_time_;
  
  if (phase_elapsed >= phase_duration_) {
    // Move to next phase
    Phase next_phase = static_cast<Phase>(static_cast<int>(current_phase_) + 1);
    
    if (next_phase <= Phase::FINISHED) {
      // Store current position as start of next phase
      for (int i = 0; i < num_joints; ++i) {
        phase_start_position_[i] = state_interfaces_[i].get_value();
      }
      
      current_phase_ = next_phase;
      phase_start_time_ = elapsed_time_;
      
      // Log phase changes
      switch (current_phase_) {
        case Phase::APPROACH_OBJECT:
          RCLCPP_INFO(rclcpp::get_logger("PickAndPlaceController"), "Phase: APPROACH_OBJECT");
          break;
        case Phase::GRASP:
          RCLCPP_INFO(rclcpp::get_logger("PickAndPlaceController"), "Phase: GRASP");
          phase_duration_ = 1.0; // Shorter duration for grasping
          break;
        case Phase::LIFT:
          RCLCPP_INFO(rclcpp::get_logger("PickAndPlaceController"), "Phase: LIFT");
          phase_duration_ = 2.0;
          break;
        case Phase::MOVE_TO_PLACE:
          RCLCPP_INFO(rclcpp::get_logger("PickAndPlaceController"), "Phase: MOVE_TO_PLACE");
          phase_duration_ = 4.0; // Longer duration for larger movement
          break;
        case Phase::PLACE:
          RCLCPP_INFO(rclcpp::get_logger("PickAndPlaceController"), "Phase: PLACE");
          phase_duration_ = 2.0;
          break;
        case Phase::RELEASE:
          RCLCPP_INFO(rclcpp::get_logger("PickAndPlaceController"), "Phase: RELEASE");
          phase_duration_ = 1.0;
          break;
        case Phase::RETURN_HOME:
          RCLCPP_INFO(rclcpp::get_logger("PickAndPlaceController"), "Phase: RETURN_HOME");
          phase_duration_ = 3.0;
          break;
        case Phase::FINISHED:
          RCLCPP_INFO(rclcpp::get_logger("PickAndPlaceController"), "Pick and place sequence completed!");
          break;
        default:
          break;
      }
    }
  }
}

void PickAndPlaceController::setTargetPositions() {
  switch (current_phase_) {
    case Phase::MOVE_TO_PICK:
      current_target_ = pick_position_;
      break;
    case Phase::APPROACH_OBJECT:
      // Move slightly closer to the object
      current_target_ = pick_position_;
      current_target_[1] += 0.1; // Move joint 2 slightly
      break;
    case Phase::GRASP:
      // Close gripper position (simulate grasping)
      current_target_ = pick_position_;
      break;
    case Phase::LIFT:
      // Lift the object by moving upward
      current_target_ = pick_position_;
      current_target_[3] -= 0.2; // Lift by adjusting joint 4
      break;
    case Phase::MOVE_TO_PLACE:
      current_target_ = place_position_;
      break;
    case Phase::PLACE:
      // Lower the object
      current_target_ = place_position_;
      current_target_[3] += 0.1; // Lower by adjusting joint 4
      break;
    case Phase::RELEASE:
      // Open gripper (same position, just release)
      current_target_ = place_position_;
      break;
    case Phase::RETURN_HOME:
      current_target_ = home_position_;
      break;
    case Phase::FINISHED:
      current_target_ = home_position_;
      break;
  }
}

void PickAndPlaceController::calculateTrajectory(const std::array<double, 7>& start_pos,
                                               const std::array<double, 7>& end_pos,
                                               double t, double duration,
                                               std::array<double, 7>& result) {
  if (duration <= 0.0 || t >= duration) {
    result = end_pos;
    return;
  }
  
  // Use a smooth trajectory (fifth-order polynomial)
  double s = t / duration;
  double smooth_s = 6.0 * std::pow(s, 5) - 15.0 * std::pow(s, 4) + 10.0 * std::pow(s, 3);
  
  for (int i = 0; i < num_joints; ++i) {
    result[i] = start_pos[i] + smooth_s * (end_pos[i] - start_pos[i]);
  }
}

CallbackReturn PickAndPlaceController::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("robot_description", "");
    auto_declare<double>("phase_duration", 3.0);
    auto_declare<double>("approach_distance", 0.05);
    auto_declare<double>("lift_height", 0.1);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn PickAndPlaceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  is_gazebo_ = get_node()->get_parameter("gazebo").as_bool();
  phase_duration_ = get_node()->get_parameter("phase_duration").as_double();
  approach_distance_ = get_node()->get_parameter("approach_distance").as_double();
  lift_height_ = get_node()->get_parameter("lift_height").as_double();

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  RCLCPP_INFO(get_node()->get_logger(), "Pick and place controller configured for arm: %s", arm_id_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn PickAndPlaceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  phase_start_time_ = 0.0;
  current_phase_ = Phase::MOVE_TO_PICK;
  
  RCLCPP_INFO(get_node()->get_logger(), "Pick and place controller activated");
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::PickAndPlaceController,
                       controller_interface::ControllerInterface)