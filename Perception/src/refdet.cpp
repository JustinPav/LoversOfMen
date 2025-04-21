#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ObjectDetectionNode : public rclcpp::Node {
public:
  ObjectDetectionNode()
  : Node("object_detection_node") {
    // Declare and get reference image parameter
    this->declare_parameter<std::string>("ref_image_path", "");
    ref_path_ = this->get_parameter("ref_image_path").as_string();

    // Load and preprocess reference image
    cv::Mat ref = cv::imread(ref_path_, cv::IMREAD_GRAYSCALE);
    if (ref.empty()) {
      RCLCPP_FATAL(this->get_logger(), "Cannot load reference image: %s", ref_path_.c_str());
      rclcpp::shutdown();
      return;
    }
    orb_ = cv::ORB::create(1000);
    orb_->detectAndCompute(ref, cv::noArray(), kp_ref_, des_ref_);
    ref_corners_ = std::vector<cv::Point2f>{
      {0, 0},
      {static_cast<float>(ref.cols), 0},
      {static_cast<float>(ref.cols), static_cast<float>(ref.rows)},
      {0, static_cast<float>(ref.rows)}
    };

    // Subscribe to color image topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&ObjectDetectionNode::imageCallback, this, std::placeholders::_1)
    );
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    // Convert ROS image to OpenCV
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // Detect features in frame
    std::vector<cv::KeyPoint> kp_frame;
    cv::Mat des_frame;
    orb_->detectAndCompute(gray, cv::noArray(), kp_frame, des_frame);

    if (!des_frame.empty() && kp_frame.size() >= 10) {
      // KNN match and ratio test
      std::vector<std::vector<cv::DMatch>> knn_matches;
      bf_.knnMatch(des_ref_, des_frame, knn_matches, 2);
      std::vector<cv::DMatch> good_matches;
      for (const auto& m : knn_matches) {
        if (m.size() == 2 && m[0].distance < 0.75f * m[1].distance) {
          good_matches.push_back(m[0]);
        }
      }

      // If enough good matches, compute homography
      if (good_matches.size() > 15) {
        std::vector<cv::Point2f> src_pts, dst_pts;
        for (const auto& match : good_matches) {
          src_pts.push_back(kp_ref_[match.queryIdx].pt);
          dst_pts.push_back(kp_frame[match.trainIdx].pt);
        }
        cv::Mat H = cv::findHomography(src_pts, dst_pts, cv::RANSAC, 5.0);
        if (!H.empty()) {
          std::vector<cv::Point2f> dst_corners;
          cv::perspectiveTransform(ref_corners_, dst_corners, H);
          std::vector<cv::Point> polygon;
          for (const auto& p : dst_corners) {
            polygon.emplace_back(static_cast<int>(p.x), static_cast<int>(p.y));
          }
          cv::polylines(frame, polygon, true, cv::Scalar(0, 255, 0), 3);
        }
      }
    }

    // Display result
    cv::imshow("Object Detection", frame);
    cv::waitKey(1);
  }

  std::string ref_path_;
  cv::Ptr<cv::ORB> orb_;
  std::vector<cv::KeyPoint> kp_ref_;
  cv::Mat des_ref_;
  std::vector<cv::Point2f> ref_corners_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  cv::BFMatcher bf_{cv::NORM_HAMMING};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetectionNode>());
  rclcpp::shutdown();
  return 0;
}