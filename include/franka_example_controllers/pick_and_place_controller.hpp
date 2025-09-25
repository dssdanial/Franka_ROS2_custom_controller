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

#pragma once

#include <string>
#include <array>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "franka_semantic_components/franka_robot_state.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The pick and place controller performs a complete pick and place sequence.
 * It moves through different phases: approach, grasp, lift, move, place, release, and return.
 */
class PickAndPlaceController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  enum class Phase {
    MOVE_TO_PICK,
    APPROACH_OBJECT,
    GRASP,
    LIFT,
    MOVE_TO_PLACE,
    PLACE,
    RELEASE,
    RETURN_HOME,
    FINISHED
  };

  // Helper functions
  void calculateTrajectory(const std::array<double, 7>& start_pos,
                          const std::array<double, 7>& end_pos,
                          double t, double duration,
                          std::array<double, 7>& result);
  void updatePhase();
  void setTargetPositions();

  // Robot parameters
  std::string arm_id_;
  bool is_gazebo_{false};
  std::string robot_description_;
  const int num_joints = 7;

  // Joint positions
  std::array<double, 7> initial_q_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> current_target_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> home_position_{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
  std::array<double, 7> pick_position_{0.2, -0.5, 0, -2.5, 0, 2.0, 0.8};
  std::array<double, 7> place_position_{-0.2, -0.3, 0, -2.2, 0, 1.8, -0.8};

  // Time management
  double elapsed_time_ = 0.0;
  double phase_start_time_ = 0.0;
  double initial_robot_time_ = 0.0;
  double robot_time_ = 0.0;
  double trajectory_period_ = 0.001;
  bool initialization_flag_{true};

  // Phase management
  Phase current_phase_{Phase::MOVE_TO_PICK};
  double phase_duration_ = 3.0; // Default duration for each phase in seconds
  std::array<double, 7> phase_start_position_{0, 0, 0, 0, 0, 0, 0};

  // Parameters
  double approach_distance_ = 0.05; // Distance to approach before grasping
  double lift_height_ = 0.1; // Height to lift after grasping
};

}  // namespace franka_example_controllers