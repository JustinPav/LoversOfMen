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
    void setBlockPoses(const std::vector<geometry_msgs::msg::Pose> &initial_poses, const std::vector<geometry_msgs::msg::Pose> &goal_poses);

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
    std::vector<geometry_msgs::msg::Pose> goal_locations_;

    // Poses used in the task stages:
    geometry_msgs::msg::Pose current_box_pose_;  // Approach pose above the block (for pick-up)
    geometry_msgs::msg::Pose current_goal_pose_; // Pose for moving toward the placement location
};

#endif // MTC_TASK_HPP
