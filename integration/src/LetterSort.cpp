#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <vector>

using std::placeholders::_1;

std::unordered_map<char, int> countLetters(const std::string& str) {
    std::unordered_map<char, int> freq;
    for (char c : str) {
        freq[c]++;
    }
    return freq;
}

bool canFormWord(const std::string& word, const std::unordered_map<char, int>& letterFreq) {
    auto wordFreq = countLetters(word);
    for (const auto& [c, count] : wordFreq) {
        if (letterFreq.find(c) == letterFreq.end() || letterFreq.at(c) < count) {
            return false;
        }
    }
    return true;
}

class LetterPosePublisher : public rclcpp::Node {
public:
    LetterPosePublisher() : Node("letter_pose_publisher") {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("letter_poses", 10);

        // 1. Get user input
        std::string letters;
        std::cout << "Enter any number of letters: ";
        std::cin >> letters;
        std::transform(letters.begin(), letters.end(), letters.begin(), ::tolower);

        auto letterFreq = countLetters(letters);

        // 2. Load dictionary
        std::ifstream dictFile("/usr/share/dict/words");
        if (!dictFile) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open dictionary file.");
            return;
        }

        std::string word;
        std::string longestWord = "";

        while (std::getline(dictFile, word)) {
            std::transform(word.begin(), word.end(), word.begin(), ::tolower);
            if (word.length() > letters.length()) continue;
            if (canFormWord(word, letterFreq) && word.length() > longestWord.length()) {
                longestWord = word;
            }
        }

        if (longestWord.empty()) {
            RCLCPP_INFO(this->get_logger(), "No valid word could be formed.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Longest word: %s", longestWord.c_str());

        // 3. Create PoseArray
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = this->now();
        pose_array.header.frame_id = "map";  // Adjust if using a different frame

        for (size_t i = 0; i < longestWord.length(); ++i) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = static_cast<float>(i) * 0.5;  // Space letters 0.5m apart
            pose.position.y = 0.0;
            pose.position.z = 0.0;

            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 1.0;

            pose_array.poses.push_back(pose);
        }

        // 4. Publish
        publisher_->publish(pose_array);
        RCLCPP_INFO(this->get_logger(), "Published PoseArray with %ld poses.", pose_array.poses.size());
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LetterPosePublisher>());
    rclcpp::shutdown();
    return 0;
}
