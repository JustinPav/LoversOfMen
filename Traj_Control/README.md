# MoveIt Task Constructor (MTC) - Block Manipulation Subsytem Test

This package demonstrates block manipulation using the MoveIt Task Constructor (MTC) with a UR3e robot and an RG2 gripper in a simulated environment.

---

## Prerequisites

Ensure you have built the workspace using the following command:

```bash
colcon build --continue-on-error
```
## Launch Instructions
Open separate terminals for each of the following commands:

1. Start the Robot Simulation
```bash
ros2 launch ur3e_rg2_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 use_fake_hardware:=true
```
2. Launch MoveIt Configuration
```bash
ros2 launch ur3e_rg2_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2
```
3. Launch the MTC Task
```bash
ros2 launch mtc mtc.launch.py ur_type:=ur3e onrobot_type:=rg2
```
## Publish Block Poses

In a new terminal, publish the initial block poses:
```bash
ros2 topic pub /initial_block_poses geometry_msgs/msg/PoseArray "poses:
- position: {x: 0.2, y: -0.20, z: 0.025}
  orientation: {w: 1.0}
- position: {x: 0.31, y: -0.29, z: 0.025}
  orientation: {w: 1.0}
- position: {x: 0.06, y: -0.36, z: 0.025}
  orientation: {w: 1.0}
- position: {x: 0.16, y: -0.42, z: 0.025}
  orientation: {w: 1.0}"
```

Then, publish the goal block poses:
```bash
ros2 topic pub /goal_block_poses geometry_msgs/msg/PoseArray "poses:
- position: {x: 0.14, y: 0.35, z: 0.025}
  orientation: {w: 1.0}
- position: {x: 0.20, y: 0.35, z: 0.025}
  orientation: {w: 1.0}
- position: {x: 0.26, y: 0.35, z: 0.025}
  orientation: {w: 1.0}
- position: {x: 0.32, y: 0.35, z: 0.025}
  orientation: {w: 1.0}"
```



## Notes
- The initial block poses are positioned in front of the robot (along +X) within a 0.4m radius.
- The goal block poses are positioned to the right of the robot.
Follow these steps to observe the robot performing the task of moving blocks from their initial positions to the goal positions.

