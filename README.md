# Franka_ROS2_custom_controller
Franka manipulator using a custom control manager to handle a Cartesian pick-and-place task.

### How to run:
Replace this package with the franka_example_controllers, compile, and build it.

Bring up the Franka robot with the following command:
```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py arm_id:=fr3 load_gripper:=true franka_hand:='franka_hand' robot_ip:=dont-care use_fake_fardware:=true use_rviz:=true

```

Then, use the bash file located in the following directory to apply the controller:

```bash
./franka_ros2/franka_example_controllers/test_pick_and_place.sh

```
