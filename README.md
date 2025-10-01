# Franka_ROS2_custom_controller
Franka manipulator using a custom control manager to handle a Cartesian pick-and-place task.

### How to run:
Replace this package with the franka_example_controllers, compile, and build it.

Bring up the Franka robot with the following command:
```bash
ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py arm_id:=fr3 load_gripper:=true franka_hand:='franka_hand' robot_ip:=dont-care use_fake_hardware:=true use_rviz:=true

```

Then, use the bash file located in the following directory to apply the controller:

```bash
./franka_ros2/franka_example_controllers/test_pick_and_place.sh

```


## Result
[Screencast from 09-26-2025 12:58:01 AM.webm](https://github.com/user-attachments/assets/f08e6457-a4d5-4e68-9d2f-61576b931760)


## Troubleshooting
If the controller did not work, try loading it manually:

```
ros2 control load_controller joint_velocity_example_controller // Or other controller like PickAndPlaceController
```
Then
```
ros2 run controller_manager spawner joint_velocity_example_controller

```
Also, if it is not activated, do it with:
```
ros2 control load_controller joint_velocity_example_controller --set-state active
```


###Note: the Gazebo simulation does NOT have Cartesian velocity command interfaces. You only have:
```
Joint position interfaces
Joint velocity interfaces
```
