#!/bin/bash

# Test script for pick and place controller

echo "Setting controller type parameter..."
ros2 param set /controller_manager pick_and_place_controller.type franka_example_controllers/PickAndPlaceController

echo "Setting controller parameters..."
ros2 param set /controller_manager pick_and_place_controller.gazebo true
ros2 param set /controller_manager pick_and_place_controller.phase_duration 3.0
ros2 param set /controller_manager pick_and_place_controller.approach_distance 0.05
ros2 param set /controller_manager pick_and_place_controller.lift_height 0.1

echo "Loading controller..."
ros2 control load_controller --set-state inactive pick_and_place_controller

echo "Activating controller..."
ros2 control set_controller_state pick_and_place_controller active

echo "Pick and place controller should now be running!"
echo "Check controller status with:"
echo "  ros2 control list_controllers"