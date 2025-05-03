#include "mtc_task.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

  // Example: set new block poses via the setBlockPoses method.
  // These can be read from user input, a file, or another source.
  geometry_msgs::msg::Pose initial_pose;
  initial_pose.position.x = 0.1;
  initial_pose.position.y = -0.4;
  initial_pose.position.z = 0.05;
  initial_pose.orientation.w = 1.0;

  std::vector<geometry_msgs::msg::Pose> initial_poses;
  initial_poses.push_back(initial_pose);

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 0.35;
  goal_pose.position.y = 0.25;
  goal_pose.position.z = 0.05;
  goal_pose.orientation.w = 1.0;

  std::vector<geometry_msgs::msg::Pose> goal_poses;
  goal_poses.push_back(goal_pose);

  // Set the new block poses
  mtc_task_node->setBlockPoses(initial_poses, goal_poses);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]()
                                                   {
      executor.add_node(mtc_task_node->getNodeBaseInterface());
      executor.spin();
      executor.remove_node(mtc_task_node->getNodeBaseInterface()); });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
