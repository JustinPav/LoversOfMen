#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"

class PosePublisher : public rclcpp::Node
{
public:
  PosePublisher()
  : Node("pose_publisher")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("detected_objects", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PosePublisher::publish_poses, this));
  }

private:
  void publish_poses()
  {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = this->get_clock()->now();
    pose_array.header.frame_id = "camera_frame";

    // Example: create 3 poses
    std::vector<std::tuple<double, double, double, double>> objects = {
      {1.0, 2.0, 0.0, 0.0}, // x, y, z, theta
      {2.0, 1.0, 0.0, 1.57},
      {3.0, 3.0, 0.0, 3.14}
    };

    for (const auto &[x, y, z, theta] : objects)
    {
      geometry_msgs::msg::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;

      tf2::Quaternion q;
      q.setRPY(0, 0, theta);  // Convert theta (yaw) to quaternion
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();

      pose_array.poses.push_back(pose);
    }

    publisher_->publish(pose_array);
    RCLCPP_INFO(this->get_logger(), "Published PoseArray with %zu poses", pose_array.poses.size());
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosePublisher>());
  rclcpp::shutdown();
  return 0;
}
