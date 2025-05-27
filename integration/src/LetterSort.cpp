#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>

#include <fstream>
#include <string>
#include <unordered_map>
#include <algorithm>

class LetterSortNode : public rclcpp::Node
{
public:
    LetterSortNode()
        : Node("letter_sort_node")
    {
        // Publisher for solved word
        word_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/initial_poses", 10);

        // Subscriber for letter 'A' poses
        sub_A_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/A", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg)
            { onPoseReceived(*msg); });
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr word_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_A_;

    bool received_A_{false};

    // Count letter frequencies
    std::unordered_map<char, int> countLetters(const std::string &str)
    {
        std::unordered_map<char, int> freq;
        for (char c : str)
            freq[c]++;
        return freq;
    }

    bool canFormWord(const std::string &word, const std::unordered_map<char, int> &freq)
    {
        auto wf = countLetters(word);
        for (auto &p : wf)
        {
            if (!freq.count(p.first) || freq.at(p.first) < p.second)
                return false;
        }
        return true;
    }

    void onPoseReceived(const geometry_msgs::msg::Pose &pose)
    {
        // Ensure orientation.w = 1
        geometry_msgs::msg::Pose p = pose;
        p.orientation.x = 0.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        p.orientation.w = 1.0;
        received_A_ = true;
        RCLCPP_INFO(get_logger(), "Received pose on /A");
        solveAndPublish();
    }

    void solveAndPublish()
    {
        if (!received_A_)
            return;

        // Only letter available is 'A'
        std::string letters = "a";
        auto freq = countLetters(letters);

        // Solve longest word from dictionary
        std::ifstream dict("/usr/share/dict/words");
        std::string word, best;
        while (std::getline(dict, word))
        {
            std::transform(word.begin(), word.end(), word.begin(), ::tolower);
            if (word.size() > letters.size())
                continue;
            if (canFormWord(word, freq) && word.size() > best.size())
            {
                best = word;
            }
        }
        if (best.empty())
        {
            RCLCPP_WARN(get_logger(), "No valid word could be formed from 'A'");
            return;
        }
        RCLCPP_INFO(get_logger(), "Solved word: %s", best.c_str());

        // Publish solved word
        std_msgs::msg::String msg;
        msg.data = best;
        word_pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LetterSortNode>());
    rclcpp::shutdown();
    return 0;
}