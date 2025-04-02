#ifndef MTC_TASK_HPP
#define MTC_TASK_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <map>

namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  // Returns the node's base interface for spinning.
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  // Setup the planning scene (e.g., add boxes at pickup locations)
  void setupPlanningScene();

  // Execute the pick and place task organized into stages.
  void doTask();

  // Set the block's initial (pickup) and goal (placement) poses.
  void setBlockPoses(const geometry_msgs::msg::Pose &initial_pose, const geometry_msgs::msg::Pose &goal_pose);

private:
  // Create an MTC task composed of several stages.
  mtc::Task createTask();

  // Helper functions for collision objects
  std::vector<moveit_msgs::msg::CollisionObject> createCollisionObjects(
      const std::vector<geometry_msgs::msg::Pose> &waypoints, const std::string &planning_frame);

  // Member variables
  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::vector<geometry_msgs::msg::Pose> block_locations_;

  // Poses used in the task stages:
  geometry_msgs::msg::Pose current_box_pose_; // Approach pose above the block (for pick-up)
  geometry_msgs::msg::Pose lifted_pose_;      // Pose after lifting the block
  geometry_msgs::msg::Pose reorient_pose_;    // Pose with the block re-oriented
  geometry_msgs::msg::Pose goal_pose_;        // Pose for moving toward the placement location
  geometry_msgs::msg::Pose place_pose_;       // Final pose for placing the block
};

#endif // MTC_TASK_HPP
