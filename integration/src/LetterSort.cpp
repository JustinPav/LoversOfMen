#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <iostream>
#include <string>
#include <unordered_map>
#include <fstream>
#include <algorithm>
#include <vector>

// Node subscribes to /F, /A, /I (Pose messages), solves longest word, then publishes
// initial poses reordered to match the solved word on /initial_block_poses as a PoseArray

class LetterSortNode : public rclcpp::Node {
public:
    LetterSortNode()
    : Node("letter_sort_node")
    {
        // Publisher for reordered initial block poses
        init_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/initial_block_poses", 10);

        // Subscribers for each letter topic
        sub_F_ = create_subscription<geometry_msgs::msg::Pose>(
            "/F", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg){ onPoseReceived('F', *msg); }
        );
        sub_A_ = create_subscription<geometry_msgs::msg::Pose>(
            "/A", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg){ onPoseReceived('A', *msg); }
        );
        sub_I_ = create_subscription<geometry_msgs::msg::Pose>(
            "/I", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg){ onPoseReceived('I', *msg); }
        );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr init_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_F_, sub_A_, sub_I_;

    std::unordered_map<char, geometry_msgs::msg::Pose> pose_map_;
    std::unordered_map<char, bool> received_{{'F',false},{'A',false},{'I',false}};

    // Count letter frequencies
    std::unordered_map<char, int> countLetters(const std::string &str) {
        std::unordered_map<char,int> freq;
        for (char c: str) freq[c]++;
        return freq;
    }
    bool canFormWord(const std::string &word, const std::unordered_map<char,int>& freq) {
        auto wf = countLetters(word);
        for (auto &p: wf) {
            char c = p.first;
            if (!freq.count(c) || freq.at(c) < p.second) return false;
        }
        return true;
    }

    void onPoseReceived(char letter, const geometry_msgs::msg::Pose &pose) {
        // Store pose, ensure orientation.w = 1
        geometry_msgs::msg::Pose p = pose;
        p.orientation.x = 0.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        p.orientation.w = 1.0;
        pose_map_[letter] = p;
        received_[letter] = true;
        RCLCPP_INFO(get_logger(), "Received pose for letter '%c'", letter);

        // Check if all received
        if (received_['F'] && received_['A'] && received_['I']) {
            solveAndPublish();
        }
    }

    void solveAndPublish() {
        // Build input letter string
        std::string letters;
        for (auto &kv: received_) {
            if (kv.second) letters.push_back(kv.first);
        }
        std::transform(letters.begin(), letters.end(), letters.begin(), ::tolower);

        // Solve longest word from letters
        auto freq = countLetters(letters);
        std::ifstream dict("/usr/share/dict/words");
        std::string word, best;
        while (std::getline(dict, word)) {
            std::transform(word.begin(), word.end(), word.begin(), ::tolower);
            if (word.size() > letters.size()) continue;
            if (canFormWord(word, freq) && word.size() > best.size()) {
                best = word;
            }
        }
        if (best.empty()) {
            RCLCPP_WARN(get_logger(), "No valid word could be formed from letters '%s'", letters.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "Solved word: %s", best.c_str());

        // Prepare PoseArray to publish
        geometry_msgs::msg::PoseArray out;
        out.header.stamp = now();
        out.header.frame_id = "map";

        // For each letter in solved word, append corresponding initial pose
        for (char c: best) {
            char U = std::toupper(c);
            if (pose_map_.count(U)) {
                out.poses.push_back(pose_map_[U]);
            }
        }

        init_pub_->publish(out);
        RCLCPP_INFO(get_logger(), "Published %zu initial poses for word '%s'", out.poses.size(), best.c_str());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LetterSortNode>());
    rclcpp::shutdown();
    return 0;
}
