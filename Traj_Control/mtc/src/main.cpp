#include "mtc_task.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <thread>

class MTCMainNode : public rclcpp::Node
{
public:
  MTCMainNode() : Node("mtc_main_node")
  {
    mtc_task_node_ = std::make_shared<MTCTaskNode>(rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Subscribers
    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/initial_block_poses", 10,
        std::bind(&MTCMainNode::initialPoseCallback, this, std::placeholders::_1));

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/goal_block_poses", 10,
        std::bind(&MTCMainNode::goalPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting for both initial and goal block poses...");
  }

  std::shared_ptr<MTCTaskNode> getMtcTaskNode() const
  {
    return mtc_task_node_;
  }

private:
  void initialPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    initial_poses_ = msg->poses;
    initial_received_ = true;
    maybeStartTask();
  }

  void goalPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (!goal_received_)
    {
      goal_poses_ = msg->poses;
      goal_received_ = true;
      total_no_of_goals_ = static_cast<int>(goal_poses_.size());
      mtc_task_node_->setBlockPoses(initial_poses_, goal_poses_);
      RCLCPP_INFO(this->get_logger(), "Received %i goal poses.", total_no_of_goals_);
    }
    maybeStartTask();
  }

  void maybeStartTask()
  {
    if (initial_received_ && goal_received_ && !task_finished_)
    {
      while (!task_finished_)
      {
        RCLCPP_INFO(this->get_logger(), "Both pose arrays received. Starting task...");

        int current_goal = total_no_of_goals_ - static_cast<int>(goal_poses_.size());
        mtc_task_node_->setupPlanningScene(current_goal);
        mtc_task_node_->doTask();

        // Remove the first goal pose from the list
        goal_poses_.erase(goal_poses_.begin());
        RCLCPP_INFO(this->get_logger(), "Goal Pose achieved. %i poses remaining.", static_cast<int>(goal_poses_.size()));

        if (goal_poses_.empty())
        {
          RCLCPP_INFO(this->get_logger(), "No goal poses remaining. Task finished.");
          task_finished_ = true;
        }
      }
    }
  }

  std::shared_ptr<MTCTaskNode> mtc_task_node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goal_pose_sub_;

  std::vector<geometry_msgs::msg::Pose> initial_poses_;
  std::vector<geometry_msgs::msg::Pose> goal_poses_;
  bool initial_received_ = false;
  bool goal_received_ = false;
  bool task_finished_ = false;
  int total_no_of_goals_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto mtc_main_node = std::make_shared<MTCMainNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mtc_main_node);
  executor.add_node(mtc_main_node->getMtcTaskNode());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
