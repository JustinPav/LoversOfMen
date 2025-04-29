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

    // Define a default pickup location (can be overridden later with setBlockPoses)
    geometry_msgs::msg::Pose p1;
    p1.position.x = 0.4;
    p1.position.y = -0.2;
    p1.position.z = 0.05;
    p1.orientation.w = 1.0;
    block_locations_.push_back(p1);

    // Set default task poses based on p1.
    // current_box_pose_ is the approach pose above the block.
    tf2::Quaternion q;
    q.setRPY(0, M_PI, 0);
    current_box_pose_ = p1;
    current_box_pose_.position.z += 0.1; // 0.1 m above the block
    current_box_pose_.orientation = tf2::toMsg(q);

    // lifted_pose_: after lifting the block vertically by 0.15 m
    lifted_pose_ = current_box_pose_;
    lifted_pose_.position.z += 0.15;

    // reorient_pose_: set to the lifted pose (adjust orientation if needed)
    reorient_pose_ = lifted_pose_;
    // For example, you can set a new orientation if required:
    // tf2::Quaternion q2;
    // q2.setRPY(M_PI/2, 0, 0);
    // reorient_pose_.orientation = tf2::toMsg(q2);

    // goal_pose_: a pose that moves the block toward a placement area.
    goal_pose_ = reorient_pose_;
    goal_pose_.position.y += 0.45; // shift in y

    // place_pose_: final placement pose (assumed to be at table height, same z as current_box_pose_)
    place_pose_ = goal_pose_;
    place_pose_.position.z = current_box_pose_.position.z;
}

// New function: setBlockPoses
void MTCTaskNode::setBlockPoses(const geometry_msgs::msg::Pose &initial_pose, const geometry_msgs::msg::Pose &goal_pose)
{
    // Update block_locations_ (for collision objects) and task poses.
    block_locations_.clear();
    block_locations_.push_back(initial_pose);

    // Set the current_box_pose_ to be an approach pose above the block.
    tf2::Quaternion q;
    q.setRPY(0, M_PI, 0);
    current_box_pose_ = initial_pose;
    current_box_pose_.position.z += 0.24; // e.g., 240 mm above the block
    current_box_pose_.orientation = tf2::toMsg(q);

    // For lifted_pose, add an additional offset.
    lifted_pose_ = current_box_pose_;
    lifted_pose_.position.z += 0.1;

    // Reorient pose can be left as lifted_pose or modified.
    reorient_pose_ = lifted_pose_;

    // Set the goal and place poses using the provided goal_pose.
    goal_pose_ = goal_pose;
    // For place_pose, we assume table height is same as the block's z (or adjust as needed)
    place_pose_ = goal_pose;
    place_pose_.position.z = current_box_pose_.position.z;
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
    task.stages()->setName("Pick and Place Task");
    task.loadRobotModel(node_);

    // Set common task properties: update these to your robot's settings.
    const std::string arm_group = "ur_onrobot_manipulator";
    const std::string eef_group = "ur_onrobot_gripper";
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

    // Stage: Move to a specific joint configuration (starting configuration)
    mtc::Stage *move_to_start_ptr = nullptr; // Forward move_to_start on to place pose generator
    auto move_to_start = std::make_unique<mtc::stages::MoveTo>("starting configuration", interpolation_planner);
    move_to_start->setGroup(arm_group);
    std::map<std::string, double> joint_targets;
    joint_targets["shoulder_pan_joint"] = 0.0;
    joint_targets["shoulder_lift_joint"] = -M_PI / 3; // -60 deg
    joint_targets["elbow_joint"] = M_PI / 3;          // 60 deg
    joint_targets["wrist_1_joint"] = -M_PI / 2;       // -90 deg
    joint_targets["wrist_2_joint"] = -M_PI / 2;       // -90 deg
    joint_targets["wrist_3_joint"] = 0.0;
    move_to_start->setGoal(joint_targets);
    move_to_start_ptr = move_to_start.get();
    task.add(std::move(move_to_start));

    // Stage: Move to block (approach the block)
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
    // clang-format on
    stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_pick));

    mtc::Stage *attach_object_stage = nullptr; // Forward attach_object_stage to place pose generator

    {
        auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
        // clang-format off
        grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                              { "eef", "group", "ik_frame" });
        // clang-format on

        {
            auto stage =
                std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
            // clang-format on
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", ik_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.1, 0.15);

            // Set hand forward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = ik_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        {
            // Sample grasp pose
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose("open");
            stage->setObject("cube_0");
            stage->setAngleDelta(M_PI / 2);
            stage->setMonitoredStage(move_to_start_ptr); // Hook into starting state

            // This is the transform from the object frame to the end-effector frame
            Eigen::Isometry3d grasp_frame_transform;
            Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
            grasp_frame_transform.linear() = q.matrix();
            grasp_frame_transform.translation().z() = 0.24;

            // Compute IK
            // clang-format off
            auto wrapper =
                std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
            // clang-format on
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(grasp_frame_transform, ik_frame);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            grasp->insert(std::move(wrapper));
        }

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject("object", ik_frame);
            attach_object_stage = stage.get();
            grasp->insert(std::move(stage));
        }
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.1, 0.3);
            stage->setIKFrame(ik_frame);
            stage->properties().set("marker_ns", "lift_object");

            // Set upward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }
        task.add(std::move(grasp));
    }

    // Stage: Re-orient box (change the end effector orientation)
    auto stage_reorient = std::make_unique<mtc::stages::MoveTo>("reorient box", interpolation_planner);
    stage_reorient->setGroup(arm_group);
    geometry_msgs::msg::PoseStamped reorient_pose_stamped;
    reorient_pose_stamped.pose = reorient_pose_;
    reorient_pose_stamped.header.frame_id = "world";
    stage_reorient->setGoal(reorient_pose_stamped);
    stage_reorient->setIKFrame(ik_frame);
    task.add(std::move(stage_reorient));

    // Stage: Move to goal pose (transport box)
    auto stage_move_to_goal = std::make_unique<mtc::stages::MoveRelative>("move to goal", cartesian_planner);
    stage_move_to_goal->properties().set("marker_ns", "move_to_goal");
    stage_move_to_goal->setGroup(arm_group);
    stage_move_to_goal->setIKFrame(ik_frame);
    stage_move_to_goal->setMinMaxDistance(0.4, 0.5); // min, max distance
    geometry_msgs::msg::Vector3Stamped vec2;
    vec2.header.frame_id = "world";
    vec2.vector.y = 1.0;
    stage_move_to_goal->setDirection(vec2);
    task.add(std::move(stage_move_to_goal));

    // Stage: Place box (final placement)
    auto stage_place = std::make_unique<mtc::stages::MoveTo>("place box", interpolation_planner);
    stage_place->setGroup(arm_group);
    geometry_msgs::msg::PoseStamped place_pose_stamped;
    place_pose_stamped.pose = place_pose_;
    place_pose_stamped.header.frame_id = "world";
    stage_place->setGoal(place_pose_stamped);
    stage_place->setIKFrame(ik_frame);
    task.add(std::move(stage_place));

    // Stage: return to starting configuration
    auto return_to_start = std::make_unique<mtc::stages::MoveTo>("starting configuration", interpolation_planner);
    return_to_start->setGroup(arm_group);
    return_to_start->setGoal(joint_targets);
    task.add(std::move(return_to_start));

    return task;
}
