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

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/configure_controller.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node = std::make_shared<rclcpp::Node>("pick_and_place_controller_node");
  
  RCLCPP_INFO(node->get_logger(), "Starting Pick and Place Controller Node");

  // Create controller manager with proper constructor
  auto controller_manager = std::make_shared<controller_manager::ControllerManager>(
      executor, "controller_manager", "", node->get_node_options());

  executor->add_node(controller_manager);

  // Create service clients for controller management
  auto load_client = node->create_client<controller_manager_msgs::srv::LoadController>(
      "/controller_manager/load_controller");
  auto configure_client = node->create_client<controller_manager_msgs::srv::ConfigureController>(
      "/controller_manager/configure_controller");  
  auto switch_client = node->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

  // Wait for services to be available
  if (!load_client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "Load controller service not available");
    return -1;
  }
  if (!configure_client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "Configure controller service not available");
    return -1;
  }
  if (!switch_client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "Switch controller service not available");
    return -1;
  }

  std::string controller_name = "pick_and_place_controller";
  std::string controller_type = "franka_example_controllers/PickAndPlaceController";

  try {
    // Load the controller
    auto load_request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
    load_request->name = controller_name;
    load_request->type = controller_type;
    
    auto load_future = load_client->async_send_request(load_request);
    if (rclcpp::spin_until_future_complete(node, load_future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Failed to call load controller service");
      return -1;
    }
    
    auto load_response = load_future.get();
    if (!load_response->ok) {
      RCLCPP_ERROR(node->get_logger(), "Failed to load controller: %s", controller_name.c_str());
      return -1;
    }

    RCLCPP_INFO(node->get_logger(), "Controller loaded successfully: %s", controller_name.c_str());

    // Configure the controller
    auto configure_request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
    configure_request->name = controller_name;
    
    auto configure_future = configure_client->async_send_request(configure_request);
    if (rclcpp::spin_until_future_complete(node, configure_future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Failed to call configure controller service");
      return -1;
    }
    
    auto configure_response = configure_future.get();
    if (!configure_response->ok) {
      RCLCPP_ERROR(node->get_logger(), "Failed to configure controller: %s", controller_name.c_str());
      return -1;
    }

    RCLCPP_INFO(node->get_logger(), "Controller configured successfully: %s", controller_name.c_str());

    // Activate the controller
    auto switch_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switch_request->activate_controllers = {controller_name};
    switch_request->deactivate_controllers = {};
    switch_request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    switch_request->activate_asap = true;
    
    auto switch_future = switch_client->async_send_request(switch_request);
    if (rclcpp::spin_until_future_complete(node, switch_future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Failed to call switch controller service");
      return -1;
    }
    
    auto switch_response = switch_future.get();
    if (!switch_response->ok) {
      RCLCPP_ERROR(node->get_logger(), "Failed to activate controller: %s", controller_name.c_str());
      return -1;
    }

    RCLCPP_INFO(node->get_logger(), "Controller activated successfully: %s", controller_name.c_str());
    RCLCPP_INFO(node->get_logger(), "Pick and place sequence starting...");

    // Run the executor
    auto start_time = node->now();
    auto timeout_duration = rclcpp::Duration::from_seconds(60.0); // 1 minute timeout
    
    while (rclcpp::ok() && (node->now() - start_time) < timeout_duration) {
      executor->spin_some(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(node->get_logger(), "Pick and place controller node shutting down");

  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in pick and place controller node: %s", e.what());
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}