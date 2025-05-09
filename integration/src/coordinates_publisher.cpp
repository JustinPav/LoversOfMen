#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <vector>

class CoordinatesPublisher : public rclcpp::Node
{
public:
    CoordinatesPublisher()
    : Node("coordinates_publisher")
    {
        // Create a publisher for PoseArray
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("coordinates", 10);

        // Initialize the timer to call the publish_coordinates function every second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CoordinatesPublisher::publish_coordinates, this)
        );

        RCLCPP_INFO(this->get_logger(), "Started coordinates publisher.");
    }

private:
    void publish_coordinates()
    {
        // Create a PoseArray message
        geometry_msgs::msg::PoseArray pose_array_msg;

        // Fill in the data for 5 objects (example)
        std::vector<std::tuple<double, double, double, double>> coordinates = {
            {1.0, 2.0, 0.5, 0.0},  // x, y, z, theta
            {2.0, 3.0, 1.0, 0.5},
            {3.0, 4.0, 1.5, 1.0},
            {4.0, 5.0, 2.0, 1.5},
            {5.0, 6.0, 2.5, 2.0}
        };

        // Populate the PoseArray message with coordinates
        for (const auto &coord : coordinates) {
            geometry_msgs::msg::Pose pose;

            // Set position (x, y, z)
            pose.position.x = std::get<0>(coord);
            pose.position.y = std::get<1>(coord);
            pose.position.z = std::get<2>(coord);

            // Set orientation (theta) - here we convert the angle to quaternion
            pose.orientation.z = std::sin(std::get<3>(coord) / 2.0); // Assuming simple 2D rotation around z-axis
            pose.orientation.w = std::cos(std::get<3>(coord) / 2.0); // Convert theta to quaternion (assuming 2D rotation)

            pose_array_msg.poses.push_back(pose); // Add this pose to the array
        }

        // Publish the PoseArray message
        publisher_->publish(pose_array_msg);

        RCLCPP_INFO(this->get_logger(), "Published a PoseArray with %zu poses.", pose_array_msg.poses.size());
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinatesPublisher>());
    rclcpp::shutdown();
    return 0;
}
