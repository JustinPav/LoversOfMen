#include "test_exec.hpp"
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

    // Define a default pickup location (can be overridden later with setBlockPoses)
    geometry_msgs::msg::Pose p1;
    p1.position.x = 0.4;
    p1.position.y = -0.2;
    p1.position.z = 0.05;
    p1.orientation.w = 1.0;
    current_box_pose_ = p1;

    geometry_msgs::msg::Pose goal1;
    goal1.position.x = 0.05;
    goal1.position.y = 0.05;
    goal1.position.z = 0.05;
    goal1.orientation.w = 1.0;

    current_goal_pose_ = goal1;
}

void MTCTaskNode::setBlockPoses(const std::vector<geometry_msgs::msg::Pose> &initial_poses, const std::vector<geometry_msgs::msg::Pose> &goal_poses)
{
    // Update block_locations_ (for collision objects) and task poses.
    block_locations_ = initial_poses;
    goal_locations_ = goal_poses;

    current_box_pose_ = block_locations_.front();
    current_goal_pose_ = goal_locations_.front();
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
    auto collision_objects = createCollisionObjects(block_locations_, "world");
    planning_scene_interface_.applyCollisionObjects(collision_objects);
    RCLCPP_INFO(node_->get_logger(), "Added %zu collision objects to the planning scene.", collision_objects.size());
}

void MTCTaskNode::doTask()
{
    task_ = createTask();

    try
    {
        task_.init();
    }
    catch (mtc::InitStageException &e)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Task initialization failed: " << e);
        return;
    }

    // Plan the task (allow up to 5 planning attempts)
    if (!task_.plan(5))
    {
        RCLCPP_ERROR(node_->get_logger(), "Task planning failed");
        return;
    }

    // Publish the first solution for introspection (if desired)
    task_.introspection().publishSolution(*task_.solutions().front());

    // Execute the task using the first solution
    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Task execution failed");
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Task executed successfully");
    return;
}

mtc::Task MTCTaskNode::createTask()
{
    mtc::Task task;
    task.stages()->setName("Test Task Execution");
    task.loadRobotModel(node_);

    // Set common task properties: update these to your robot's settings.
    const std::string arm_group = "ur_onrobot_manipulator";
    const std::string gripper_group = "ur_onrobot_gripper";
    const std::string eef_group = "ur_onrobot_tcp";
    const std::string ik_frame = "tool0";

    task.setProperty("group", arm_group);
    task.setProperty("eef", eef_group);
    task.setProperty("ik_frame", ik_frame);

    // Stage 1: Current state
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

    auto stage_current = std::make_unique<mtc::stages::CurrentState>("current state");
    current_state_ptr = stage_current.get();
    task.add(std::move(stage_current));

    // Create planners for the stages (using Pipeline or JointInterpolation planners)
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    sampling_planner->init(task.getRobotModel());
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(0.005);

    mtc::Stage *move_to_start_ptr = nullptr; // Forward move_to_start on to place pose generator
    auto move_to_start = std::make_unique<mtc::stages::MoveTo>("starting configuration", interpolation_planner);
    move_to_start->setGroup(arm_group);
    std::map<std::string, double> joint_targets;
    joint_targets["shoulder_pan_joint"] = 0.0;
    joint_targets["shoulder_lift_joint"] = -(80 * M_PI) / 180; // -80 deg
    joint_targets["elbow_joint"] = (80 * M_PI) / 180;          // 80 deg
    joint_targets["wrist_1_joint"] = -M_PI / 2;                // -90 deg
    joint_targets["wrist_2_joint"] = -M_PI / 2;                // -90 deg
    joint_targets["wrist_3_joint"] = 0.0;
    move_to_start->setGoal(joint_targets);
    move_to_start_ptr = move_to_start.get();
    task.add(std::move(move_to_start));

    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(gripper_group);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));

    auto stage_return_up = std::make_unique<mtc::stages::MoveTo>("return up", interpolation_planner);
    stage_return_up->setGroup(arm_group);
    stage_return_up->setGoal("up");
    task.add(std::move(stage_return_up));

    auto stage_close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage_close_hand->setGroup(gripper_group);
    stage_close_hand->setGoal("closed");
    task.add(std::move(stage_close_hand));

    return task;
}
