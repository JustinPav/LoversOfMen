#include "mtc_task.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mtc_task_node->getNodeBaseInterface());

  // Run the executor in a separate thread.
  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  // Setup planning scene (e.g. add collision objects)
  mtc_task_node->setupPlanningScene();

  // Execute the pick-and-place task
  mtc_task_node->doTask();

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
