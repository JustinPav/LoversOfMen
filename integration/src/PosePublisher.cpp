#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <iostream>
#include <string>
#include <sstream>

class Publish : public rclcpp::Node
{
public:
    Publish()
    : Node("pose_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("detected_objects", 10);

        // Start the input loop in a separate thread so it doesn’t block ROS
        input_thread_ = std::thread([this]() { input_loop(); });
        input_thread_.detach();  // Allows ROS to shut down cleanly
    }

private:
    void input_loop()
    {
        while (rclcpp::ok()) {
            std::cout << "Enter poses in format: x y z theta (type 'done' to publish):\n";

            geometry_msgs::msg::PoseArray pose_array;
            pose_array.header.stamp = this->get_clock()->now();
            pose_array.header.frame_id = "camera_frame";

            std::string line;
            while (true) {
                std::getline(std::cin, line);
                if (line == "done") break;

                std::istringstream iss(line);
                double x, y, z, theta;
                if (!(iss >> x >> y >> z >> theta)) {
                    std::cout << "Invalid input. Try again.\n";
                    continue;
                }

                geometry_msgs::msg::Pose pose;
                pose.position.x = x;
                pose.position.y = y;
                pose.position.z = z;

                tf2::Quaternion q;
                q.setRPY(0, 0, theta);
                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();
                pose.orientation.w = q.w();

                pose_array.poses.push_back(pose);
            }

            publisher_->publish(pose_array);
            RCLCPP_INFO(this->get_logger(), "Published %zu poses", pose_array.poses.size());
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    std::thread input_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Publish>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
