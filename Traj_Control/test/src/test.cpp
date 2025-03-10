#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ur3_joint_commander");
    auto publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 10);

    // Allow time for the publisher to establish connections
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Define the joint names for the UR3
    std::vector<std::string> joint_names = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };

    // Create a JointTrajectory message
    trajectory_msgs::msg::JointTrajectory trajectory_msg;
    trajectory_msg.joint_names = joint_names;

    // Define a single point in the trajectory
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0, -1.57, 1.57, 0.0, 0.0, 0.0}; // Example joint positions in radians
    point.time_from_start = rclcpp::Duration::from_seconds(2.0); // 2-second duration

    // Add the point to the trajectory
    trajectory_msg.points.push_back(point);

    // Publish the trajectory
    publisher->publish(trajectory_msg);
    RCLCPP_INFO(node->get_logger(), "Trajectory published.");

    // Keep the node alive to ensure the message is sent
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::seconds(3));

    rclcpp::shutdown();
    return 0;
}
