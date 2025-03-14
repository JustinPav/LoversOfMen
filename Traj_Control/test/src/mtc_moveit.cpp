#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <iostream>

// Function to create collision objects (50mm cubes) at the given waypoints.
std::vector<moveit_msgs::msg::CollisionObject> createCollisionObjects(
  const std::vector<geometry_msgs::msg::Pose>& waypoints,
  const std::string& planning_frame)
{
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  for (size_t i = 0; i < waypoints.size(); ++i)
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame;
    collision_object.id = "cube_" + std::to_string(i);

    // Define the cube primitive (50mm cube = 0.05m each side)
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.05;
    primitive.dimensions[1] = 0.05;
    primitive.dimensions[2] = 0.05;

    // Set the pose of the cube to be at the given waypoint
    geometry_msgs::msg::Pose cube_pose = waypoints[i];

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cube_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);
  }
  return collision_objects;
}

// Function to update waypoints so that each is 0.1m above the corresponding box and end effector faces downward.
// Downward-facing orientation: 180° rotation about X-axis (quaternion: w=0, x=1, y=0, z=0)
std::vector<geometry_msgs::msg::Pose> updateWaypointsForGrasping(
  const std::vector<geometry_msgs::msg::Pose>& original_waypoints, double offset_z)
{
  std::vector<geometry_msgs::msg::Pose> updated_waypoints;
  for (const auto& pose : original_waypoints)
  {
    geometry_msgs::msg::Pose new_pose = pose;
    // Offset by offset_z in the Z direction (assuming the cube's z is its center; adjust if needed)
    new_pose.position.z += offset_z;
    // Set orientation for the end effector to face downward.
    // This quaternion corresponds to a 180° rotation about the X-axis.
    new_pose.orientation.w = 0.0;
    new_pose.orientation.x = 1.0;
    new_pose.orientation.y = 0.0;
    new_pose.orientation.z = 0.0;
    updated_waypoints.push_back(new_pose);
  }
  return updated_waypoints;
}

// Function to compute and execute a Cartesian path
bool planAndExecuteCartesianPath(moveit::planning_interface::MoveGroupInterface& move_group,
                                   const std::vector<geometry_msgs::msg::Pose>& waypoints,
                                   rclcpp::Logger logger)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.01;      // Resolution of the computed path (in meters)
  const double jump_threshold = 0.0; // Disable jump threshold checking
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  if (fraction > 0.9)
  {
    RCLCPP_INFO(logger, "Cartesian path computed successfully (%.2f%% achieved)", fraction * 100.0);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    auto exec_result = move_group.execute(plan);
    if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "Trajectory executed successfully.");
      return true;
    }
    else
    {
      RCLCPP_ERROR(logger, "Failed to execute the trajectory.");
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Cartesian path planning failed (only %.2f%% achieved)", fraction * 100.0);
    return false;
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit_pick_and_place_skeleton");

  // Create a MultiThreadedExecutor for asynchronous callbacks.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Create the MoveGroupInterface for your robot's planning group
  const std::string planning_group = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);

  // Create the PlanningSceneInterface to add collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Optional: set planning time, velocity scaling, etc.
  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(0.1);

  // Allow time for initialization
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Define original waypoints for the box positions
  std::vector<geometry_msgs::msg::Pose> original_waypoints;

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.position.x = 0.1;
  target_pose1.position.y = 0.4;
  target_pose1.position.z = 0.05;
  target_pose1.orientation.w = 1.0;  // Orientation is not used for the cube objects.
  original_waypoints.push_back(target_pose1);

  geometry_msgs::msg::Pose target_pose2;
  target_pose2.position.x = 0.35;
  target_pose2.position.y = -0.2;
  target_pose2.position.z = 0.05;
  target_pose2.orientation.w = 1.0;
  original_waypoints.push_back(target_pose2);

  geometry_msgs::msg::Pose target_pose3;
  target_pose3.position.x = 0.35;
  target_pose3.position.y = 0.2;
  target_pose3.position.z = 0.05;
  target_pose3.orientation.w = 1.0;
  original_waypoints.push_back(target_pose3);

  // Add collision objects (cubes) at the original waypoints
  auto collision_objects = createCollisionObjects(original_waypoints, move_group.getPlanningFrame());
  planning_scene_interface.applyCollisionObjects(collision_objects);
  RCLCPP_INFO(node->get_logger(), "Added %zu collision objects (cubes) to the planning scene.", collision_objects.size());

  // Wait for user input before proceeding
  std::cout << "Press Enter to continue with the task..." << std::endl;
  std::cin.get();

  // Update waypoints to be 0.1m above each cube and set end effector facing downward
  double grasp_offset = 0.1;
  auto grasp_waypoints = updateWaypointsForGrasping(original_waypoints, grasp_offset);

  // Plan and execute the Cartesian path based on the updated waypoints
  planAndExecuteCartesianPath(move_group, grasp_waypoints, node->get_logger());

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
