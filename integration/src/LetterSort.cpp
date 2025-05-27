#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <fstream>
#include <string>
#include <unordered_map>
#include <algorithm>

class LetterSortNode : public rclcpp::Node {
public:
    LetterSortNode()
    : Node("letter_sort_node")
    {
        // Publisher for PoseArray of solved word poses
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/initial_poses", 10);

        // Subscriber for letter 'A' point
        sub_A_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/A", 10,
            [this](const geometry_msgs::msg::PointStamped::SharedPtr msg){ onPointReceived(*msg); }
        );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_A_;

    geometry_msgs::msg::Pose last_pose_;
    bool received_A_ = false;

    // Count letter frequencies
    std::unordered_map<char,int> countLetters(const std::string &str) {
        std::unordered_map<char,int> freq;
        for (char c: str) freq[c]++;
        return freq;
    }

    bool canFormWord(const std::string &word, const std::unordered_map<char,int> &freq) {
        auto wf = countLetters(word);
        for (auto &p: wf) {
            if (!freq.count(p.first) || freq.at(p.first) < p.second) return false;
        }
        return true;
    }

    void onPointReceived(const geometry_msgs::msg::PointStamped &msg) {
        // Convert PointStamped to Pose with orientation.w = 1
        last_pose_.position = msg.point;
        last_pose_.orientation.x = 0.0;
        last_pose_.orientation.y = 0.0;
        last_pose_.orientation.z = 0.0;
        last_pose_.orientation.w = 1.0;
        received_A_ = true;
        RCLCPP_INFO(get_logger(), "Received point on /A: [%.2f, %.2f, %.2f]", 
                    msg.point.x, msg.point.y, msg.point.z);
        solveAndPublish();
    }

    void solveAndPublish() {
        if (!received_A_) return;

        // Only letter available is 'A'
        std::string letters = "a";
        auto freq = countLetters(letters);

        // Solve longest word from dictionary
        std::ifstream dict("/usr/share/dict/words");
        std::string word, best, longestWord;
        while (std::getline(dict, word)) {
            std::transform(word.begin(), word.end(), word.begin(), ::tolower);
            if (word.length() > letters.length())
                continue;
            if (canFormWord(word, freq) && word.length() > best.length())
            {
                longestWord = word;
            }
        }
        if (best.empty()) {
            RCLCPP_WARN(get_logger(), "No valid word could be formed from 'A'");
            return;
        }
        RCLCPP_INFO(get_logger(), "Solved word: %s", best.c_str());

        // Publish PoseArray for each letter in solved word
        geometry_msgs::msg::PoseArray out;
        out.header.stamp = now();
        out.header.frame_id = "map";

        for (size_t i = 0; i < best.size(); ++i) {
            out.poses.push_back(last_pose_);
        }
        pose_pub_->publish(out);
        RCLCPP_INFO(get_logger(), "Published %zu poses for word '%s'", out.poses.size(), best.c_str());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LetterSortNode>());
    rclcpp::shutdown();
    return 0;
}