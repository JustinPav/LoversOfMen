#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

class PoseSubscriber : public rclcpp::Node
{
public:
    PoseSubscriber() : Node("pose_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "detected_objects", 10,
            std::bind(&PoseSubscriber::topic_callback, this, std::placeholders::_1)
        );
    }

private:
    void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses", msg->poses.size());
        for (size_t i = 0; i < msg->poses.size(); ++i) {
            const auto& p = msg->poses[i];
            RCLCPP_INFO(this->get_logger(), "Pose %zu: Position (%.2f, %.2f, %.2f) | Orientation (%.2f, %.2f, %.2f, %.2f)",
                        i, p.position.x, p.position.y, p.position.z,
                        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseSubscriber>());
    rclcpp::shutdown();
    return 0;
}
