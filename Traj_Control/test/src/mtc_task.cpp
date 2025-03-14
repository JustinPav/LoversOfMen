#include "mtc_task.hpp"
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <thread>
#include <iostream>

using namespace std::chrono_literals;

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options)
{
    node_ = std::make_shared<rclcpp::Node>("mtc_task_node", options);

    // Define pickup waypoints for placing collision objects (e.g., boxes)
    // Here, we define three pickup locations.
    geometry_msgs::msg::Pose p1;
    p1.position.x = 0.1;
    p1.position.y = 0.4;
    p1.position.z = 0.05;
    p1.orientation.w = 1.0;
    pickup_waypoints_.push_back(p1);

    geometry_msgs::msg::Pose p2;
    p2.position.x = 0.35;
    p2.position.y = -0.2;
    p2.position.z = 0.05;
    p2.orientation.w = 1.0;
    pickup_waypoints_.push_back(p2);

    geometry_msgs::msg::Pose p3;
    p3.position.x = 0.35;
    p3.position.y = 0.2;
    p3.position.z = 0.05;
    p3.orientation.w = 1.0;
    pickup_waypoints_.push_back(p3);

    // Define dummy poses for the task stages.
    // Update these values to reflect your real task.
    // box_pose_: approach pose above the box
    box_pose_ = p1;
    box_pose_.position.z += 0.1; // 0.1 m above the box

    // lift_pose_: after lifting the box vertically by 0.1 m
    lift_pose_ = box_pose_;
    lift_pose_.position.z += 0.1;

    // reorient_pose_: change orientation (for example, face downward)
    reorient_pose_ = lift_pose_;
    // Downward-facing: 180° rotation about X-axis.
    reorient_pose_.orientation.w = 0.0;
    reorient_pose_.orientation.x = 1.0;
    reorient_pose_.orientation.y = 0.0;
    reorient_pose_.orientation.z = 0.0;

    // goal_pose_: a pose that moves the box toward a placement area.
    goal_pose_ = reorient_pose_;
    goal_pose_.position.x += 0.2; // shift in x

    // place_pose_: final placement pose.
    place_pose_ = goal_pose_;
    place_pose_.position.z = 0.3; // assume table height
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
    return node_->get_node_base_interface();
}

std::vector<moveit_msgs::msg::CollisionObject> MTCTaskNode::createCollisionObjects(
    const std::vector<geometry_msgs::msg::Pose> &waypoints, const std::string &planning_frame)
{
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = planning_frame;
        collision_object.id = "cube_" + std::to_string(i);

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        // 50 mm cube (0.05 m per side)
        primitive.dimensions[0] = 0.05;
        primitive.dimensions[1] = 0.05;
        primitive.dimensions[2] = 0.05;

        geometry_msgs::msg::Pose cube_pose = waypoints[i];

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(cube_pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);
    }
    return collision_objects;
}

void MTCTaskNode::setupPlanningScene()
{
    auto collision_objects = createCollisionObjects(pickup_waypoints_, "world");
    planning_scene_interface_.applyCollisionObjects(collision_objects);
    RCLCPP_INFO(node_->get_logger(), "Added %zu collision objects to the planning scene.", collision_objects.size());
}

mtc::Task MTCTaskNode::createTask()
{
    mtc::Task task;
    task.stages()->setName("Pick and Place Task");
    task.loadRobotModel(node_);

    // Set common task properties: update these to your robot's settings.
    const std::string arm_group = "ur_manipulator";
    const std::string eef_group = "gripper"; // Update as needed.
    const std::string ik_frame = "tool0";    // End effector frame.

    task.setProperty("group", arm_group);
    task.setProperty("eef", eef_group);
    task.setProperty("ik_frame", ik_frame);

    // Stage 1: Current state
    auto stage_current = std::make_unique<mtc::stages::CurrentState>("current state");
    task.add(std::move(stage_current));

    // Create planners for the stages (using Pipeline or JointInterpolation planners)
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    // Stage 2: Move to box (approach above the box)
    auto stage_move_to_box = std::make_unique<mtc::stages::MoveTo>("move to box", interpolation_planner);
    stage_move_to_box->setGroup(arm_group);

    geometry_msgs::msg::PoseStamped box_pose_stamped;
    box_pose_stamped.pose = box_pose_;
    box_pose_stamped.header.frame_id = "world";
    stage_move_to_box->setGoal(box_pose_stamped);
    task.add(std::move(stage_move_to_box));

    // Stage 3: Lift box (move upward relatively)
    auto stage_lift = std::make_unique<mtc::stages::MoveRelative>("lift box");
    // Define a relative translation of 0.1 m upward.
    geometry_msgs::msg::TwistStamped translation;
    translation.header.frame_id = "world";
    translation.twist.linear.x = 0.0;
    translation.twist.linear.y = 0.0;
    translation.twist.linear.z = 0.1;
    stage_lift->setDirection(translation);
    task.add(std::move(stage_lift));

    // Stage 4: Re-orient box (change the end effector orientation)
    auto stage_reorient = std::make_unique<mtc::stages::MoveTo>("reorient box", interpolation_planner);
    stage_reorient->setGroup(arm_group);

    geometry_msgs::msg::PoseStamped reorient_pose_stamped;
    reorient_pose_stamped.pose = reorient_pose_;
    reorient_pose_stamped.header.frame_id = "world";
    stage_reorient->setGoal(reorient_pose_stamped);
    task.add(std::move(stage_reorient));

    // Stage 5: Move to goal pose (transport box)
    auto stage_move_to_goal = std::make_unique<mtc::stages::MoveTo>("move to goal", interpolation_planner);
    stage_move_to_goal->setGroup(arm_group);
    geometry_msgs::msg::PoseStamped goal_pose_stamped;
    goal_pose_stamped.pose = goal_pose_;
    goal_pose_stamped.header.frame_id = "world";
    stage_move_to_goal->setGoal(goal_pose_stamped);
    task.add(std::move(stage_move_to_goal));

    // Stage 6: Place box (final placement)
    auto stage_place = std::make_unique<mtc::stages::MoveTo>("place box", interpolation_planner);
    stage_place->setGroup(arm_group);
    geometry_msgs::msg::PoseStamped place_pose_stamped;
    place_pose_stamped.pose = place_pose_;
    place_pose_stamped.header.frame_id = "world";
    stage_place->setGoal(place_pose_stamped);
    task.add(std::move(stage_place));

    return task;
}

void MTCTaskNode::doTask()
{
    // Wait for user input before proceeding.

    mtc::Task task = createTask();

    std::cout << "Press Enter to start task execution..." << std::endl;
    std::cin.get();

    try
    {
        task.init();
    }
    catch (mtc::InitStageException &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Task initialization failed: %s", e.what());
        return;
    }

    // Plan the task (allow up to 5 planning attempts)
    if (!task.plan(5))
    {
        RCLCPP_ERROR(node_->get_logger(), "Task planning failed");
        return;
    }

    // Publish the first solution for introspection (if desired)
    task.introspection().publishSolution(*task.solutions().front());

    // Execute the task using the first solution
    auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Task execution failed");
        return;
    }
    RCLCPP_INFO(node_->get_logger(), "Task executed successfully");
}
