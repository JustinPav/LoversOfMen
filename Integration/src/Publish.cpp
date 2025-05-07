#include "rclcpp/rclcpp.hpp"
#include "integration/msg/coordinates.hpp"

#include <fstream>
#include <sstream>
#include <vector>
#include <string>

class CoordinatesPublisher : public rclcpp::Node
{
public:
    CoordinatesPublisher()
    : Node("coordinates_publisher"), current_index_(0)
    {
        load_coordinates();
        publisher_ = this->create_publisher<integration::msg::Coordinates>("coordinates", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&CoordinatesPublisher::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Started coordinates publisher.");
    }

private:
    void load_coordinates()
    {
        std::ifstream file("data/coordinates.txt");  // Path relative to package root (see note below)
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open coordinates file!");
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream ss(line);
            double x, y, z;
            if (ss >> x >> y >> z) {
                coordinates_.emplace_back(x, y, z);
            }
        }

        file.close();
        RCLCPP_INFO(this->get_logger(), "Loaded %zu coordinates.", coordinates_.size());
    }

    void timer_callback()
    {
        if (coordinates_.empty()) return;

        const auto &[x, y, z] = coordinates_[current_index_];

        integration::msg::Coordinates msg;
        msg.x = x;
        msg.y = y;
        msg.z = z;
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Publishing: x=%.2f y=%.2f z=%.2f", x, y, z);

        current_index_ = (current_index_ + 1) % coordinates_.size();  // Loop over
    }

    rclcpp::Publisher<integration::msg::Coordinates>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::tuple<double, double, double>> coordinates_;
    size_t current_index_;
};
 
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinatesPublisher>());
    rclcpp::shutdown();
    return 0;
}
