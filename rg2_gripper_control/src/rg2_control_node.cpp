#include <chrono>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class RG2ControlNode : public rclcpp::Node
{
public:
    RG2ControlNode() : Node("rg2_control_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/onrobot/finger_width_controller/commands", 10);
        
        // Let ROS2 system warm up and subscribers connect
        std::this_thread::sleep_for(std::chrono::seconds(2));     
        
        // Publish open and close commands
        publish_open();
        std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait 2 seconds
        publish_close();
        std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait 2 seconds
        publish_open();
    }

private:
    void publish_open()
    {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {0.08}; // 80 mm (open)
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Opening gripper (finger_width: %.3f m)", msg.data[0]);
    }

    void publish_close()
    {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {0.05}; // 50 mm (closed)
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Closing gripper (finger_width: %.3f m)", msg.data[0]);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RG2ControlNode>();
    // No spin needed since we publish directly and exit
    rclcpp::shutdown();
    return 0;
}