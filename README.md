# Lovers of Men (LoM) - Pick & Place Alphabet Blocks

## Project Overview
The Pick-and-place project aims to utilise a robot with an end effector manipulator to identify and sort a group of items. Our project specifically identifies the initial poses of a variety of alphabet blocks and then manipulates them in order to spell a word by placing the blocks in order.

This project demonstrates robotic block manipulation using the MoveIt Task Constructor (MTC) framework with a UR3e robot and an RG2 gripper. It combines this framework with a perception node which is utilised to identify the inital locations of the blocks which is then sent to a word solver to produce the longest word with the given letters.
**Key Features:**
- Block manipulation using MoveIt Task Constructor.
- Integration with UR3e robot and RG2 gripper.
- Perception using a Realsense D435.
- Interactive UI to communicate between nodes and inspect the job progress.

**Subsystems:**
- Perception & Sensing
- Trajectory Planning & Motion Control 
- Grasping & Manipulation 
- System Integration & User Interface 

---

## Dependencies

### Hardware
- **Robot:** Universal Robots UR3e.
- **Gripper:** OnRobot RG2.
- **RGB-D Camera:** Realsense D435.
- **Alphabet Bricks:** Custom made LoM Alphabet Bricks

### Computing Specs
- **OS:** Ubuntu 22.04 (or compatible).
- **ROS Version:** ROS 2 Humble Hawksbill.
- **RAM:** Minimum 8GB recommended.

### Software
- ROS 2 Humble.
- MoveIt 2.
- ...Others...

---

## Installation

### Hardware Setup
1. Assemble the UR3e robot and RG2 gripper.
2. Ensure the workspace is clear and the robot is securely mounted.
3. Place the provided alphabet bricks in any way to the left of the arm.
4. Connect the robot to the control PC.

### Software Setup
1. Install all the required packages:
   ```bash
   sudo apt-get install ros-humble-ur
   sudo apt-get install ros-humble-moveit
   ```

2. Navigate to your workspace and **clone all the following repositories** into the `src` directory:
   ```bash
   git clone https://github.com/JustinPav/LoversOfMen.git src/lovers_of_men
   git clone https://github.com/tonydle/OnRobot_ROS2_Description.git src/onrobot_description
   git clone https://github.com/tonydle/UR_OnRobot_ROS2.git src/ur_onrobot
   git clone --recurse-submodules https://github.com/tonydle/OnRobot_ROS2_Driver.git src/onrobot_driver
   git clone https://github.com/moveit/moveit_task_constructor.git src/moveit_task_constructor
   ```
3. Build the workspace:
   ```bash
   colcon build --symlink-install
   ```
4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

---

## Running the System

### Launch Commands
1. Start the robot simulation:
   ```bash
   ros2 launch ur3e_rg2_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 robot_ip:=<robot_ip_here>
   ```
2. Launch the MoveIt configuration:
   ```bash
   ros2 launch ur3e_rg2_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2
   ```
3. Launch the MTC task:
   ```bash
   ros2 launch mtc mtc.launch.py ur_type:=ur3e onrobot_type:=rg2
   ```
4. Launch the Perception node:
    ```bash
   ros2 launch 
   ```
5. Launch the GUI:
    ```bash
   ros2 launch 
   ```

### Expected Outcome
- The robot should pick up blocks from their initial positions and place them at the goal positions.

---

## Subsystem Specifics

### Perception & Sensing

### Trajectory Planning & Motion Control 
#### Purpose
The purpose of this subsystem is to plan and execute collision-free motion trajectories for the UR3e robot during the pick-and-place task. It leverages the MoveIt Task Constructor (MTC) framework to construct a multi-stage manipulation plan that includes approaching, grasping, lifting, transporting, and placing blocks. The subsystem is responsible for integrating perception-derived poses, generating appropriate robot motions, and safely executing them on the physical robot.

#### Key Topics, Services, and Files

| Element | Description |
|--------|-------------|
| `/initial_block_poses` *(topic)* | Subscribes to initial block poses published as `geometry_msgs::msg::PoseArray`. |
| `/goal_block_poses` *(topic)* | Subscribes to goal block poses published as `geometry_msgs::msg::PoseArray`. |
| `mtc_task_node.cpp` | Main node source file that sets up the MTC task, subscribes to input poses, and triggers task execution. |
| `mtc_task.hpp` | Header file for the `MTCTaskNode` class, encapsulating the task setup, configuration, and execution logic. |
| `ur3e_moveit_config/` | MoveIt configuration package including planner settings, joint limits, and robot semantics (SRDF). |
| `launch/mtc_task.launch.py` | Launch file (if used) that starts the MTC node and supporting components such as MoveIt and RViz. |

---

### Grasping & Manipulation 

### System Integration & User Interface 


---

## Troubleshooting & FAQs

### Common Issues

### FAQs


---