#include "mtc_task.hpp"
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <thread>
#include <iostream>

using namespace std::chrono_literals;

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("mtc_task_node", options)
{

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
    for (int i = 0; i < goal_poses.size(); ++i)
    {
        auto goal = goal_poses.at(i);
        auto block = initial_poses.at(i);
        geometry_msgs::msg::Pose goal_ref;
        goal_ref.position.x = goal.position.x - block.position.x;
        goal_ref.position.y = goal.position.y - block.position.y;
        goal_ref.position.z = goal.position.z - block.position.z;
        goal_ref.orientation = goal.orientation;
        goal_locations_.push_back(goal_ref);
    }

    current_box_pose_ = block_locations_.front();
    current_goal_pose_ = goal_locations_.front();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
    return this->get_node_base_interface();
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
    RCLCPP_INFO(this->get_logger(), "Added %zu collision objects to the planning scene.", collision_objects.size());
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
        RCLCPP_ERROR_STREAM(this->get_logger(), "Task initialization failed: " << e);
        return;
    }

    // Plan the task
    if (!task_.plan(15))
    {
        RCLCPP_ERROR(this->get_logger(), "Task planning failed");
        return;
    }

    // Publish the first solution for introspection (if desired)
    task_.introspection().publishSolution(*task_.solutions().front());

    // Execute the task using the first solution
    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Task execution failed");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Task executed successfully");
    return;
}

mtc::Task MTCTaskNode::createTask()
{
    mtc::Task task;
    task.stages()->setName("Pick and Place Task");
    task.loadRobotModel(this->shared_from_this());

    // Set common task properties: update these to your robot's settings.
    const std::string arm_group = "ur_onrobot_manipulator";
    const std::string gripper_group = "ur_onrobot_gripper";
    const std::string eef_group = "ur_onrobot_tcp";
    const std::string ik_frame = "tool0";

    task.setProperty("group", arm_group);
    task.setProperty("eef", eef_group);
    task.setProperty("ik_frame", ik_frame);

    task.setCostTerm(std::make_shared<mtc::cost::TrajectoryDuration>());

    // Set timeout for Connect stages as well as acceleration and velocity scaling factors
    double timeout = 5.0;
    double acc = 0.5;
    double vel = 1.0;

    // Stage 1: Current state
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

    auto stage_current = std::make_unique<mtc::stages::CurrentState>("current state");
    current_state_ptr = stage_current.get();
    task.add(std::move(stage_current));

    // Create planners for the stages (using Pipeline or JointInterpolation planners)
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this(), "ompl");
    sampling_planner->setMaxAccelerationScalingFactor(acc);
    sampling_planner->setMaxVelocityScalingFactor(vel);

    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    interpolation_planner->setMaxAccelerationScalingFactor(acc);
    interpolation_planner->setMaxVelocityScalingFactor(vel);

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxAccelerationScalingFactor(acc);
    cartesian_planner->setMaxVelocityScalingFactor(vel);
    cartesian_planner->setStepSize(0.005);

    // Stage: Move to a specific joint configuration (starting configuration)
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

    // Stage: open gripper
    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(gripper_group);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));

    // Stage: Move to block (approach the block)
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});

    stage_move_to_pick->setTimeout(timeout);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    stage_move_to_pick->setMaxDistance(10.0);
    task.add(std::move(stage_move_to_pick));

    mtc::Stage *attach_object_stage = nullptr; // Forward attach_object_stage to place pose generator

    // Stage: Pick object
    // Create a container for the grasp stage
    {
        auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
        grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        {
            auto stage =
                std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);

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
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
            stage->allowCollisions("cube_0", task.getRobotModel()->getJointModelGroup(gripper_group)->getLinkModelNamesWithCollisionGeometry(), true);
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
            Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
            grasp_frame_transform.linear() = q.matrix();
            grasp_frame_transform.translation().z() = 0.23;

            // Compute IK

            auto wrapper =
                std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));

            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(grasp_frame_transform, ik_frame);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            grasp->insert(std::move(wrapper));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
            stage->setGroup(gripper_group);
            stage->setGoal("closed");
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject("cube_0", ik_frame);
            attach_object_stage = stage.get();
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.07, 0.3);
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

    // Stage: Move to place pose
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});

    stage_move_to_place->setTimeout(timeout);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    stage_move_to_place->setMaxDistance(10.0);
    task.add(std::move(stage_move_to_place));

    // Stage: Place object
    // Create a container for the place stage
    {
        auto place = std::make_unique<mtc::SerialContainer>("place object");
        task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
        place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        {
            // Sample place pose
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "place_pose");
            stage->setObject("cube_0");

            geometry_msgs::msg::PoseStamped target_pose_msg;
            target_pose_msg.header.frame_id = "cube_0";
            target_pose_msg.pose = current_goal_pose_;
            stage->setPose(target_pose_msg);
            stage->setMonitoredStage(attach_object_stage); // Hook into attach_object_stage

            // Compute IK

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));

            wrapper->setMaxIKSolutions(2);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame("cube_0");
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            place->insert(std::move(wrapper));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
            stage->setGroup(gripper_group);
            stage->setGoal("open");
            place->insert(std::move(stage));
        }

        {

            auto stage =
                std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision");
            stage->allowCollisions("cube_0",
                                   task.getRobotModel()
                                       ->getJointModelGroup(gripper_group)
                                       ->getLinkModelNamesWithCollisionGeometry(),
                                   false);

            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
            stage->detachObject("cube_0", ik_frame);
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.07, 0.3);
            stage->setIKFrame(ik_frame);
            stage->properties().set("marker_ns", "retreat");

            // Set retreat direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 0.5;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }
        task.add(std::move(place));
    }

    // Stage: Return home
    auto stage_return_home = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage_return_home->setGroup(arm_group);
    stage_return_home->setGoal(joint_targets);
    task.add(std::move(stage_return_home));

    return task;
}
