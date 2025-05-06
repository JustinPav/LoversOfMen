// src/object_detection_node.cpp

#include <memory>
#include <string>
#include <vector>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "perception/msg/coordinates.hpp" // Include the custom message


class ObjectDetectionNode : public rclcpp::Node {
public:
  ObjectDetectionNode()
  : Node("object_detection_node")
  {
    // 1) Declare & read the reference-image path
    this->declare_parameter<std::string>("ref_image_path", "");
    ref_path_ = this->get_parameter("ref_image_path").as_string();

    if (ref_path_.empty()) {
      RCLCPP_FATAL(get_logger(),
        "Parameter 'ref_image_path' is empty. Pass it via -p ref_image_path:=/full/path.jpg");
      rclcpp::shutdown();
      return;
    }

    // 2) Load & preprocess reference image
    RCLCPP_INFO(get_logger(), "Loading reference image from: %s", ref_path_.c_str());
    cv::Mat ref_gray = cv::imread(ref_path_, cv::IMREAD_GRAYSCALE);
    if (ref_gray.empty()) {
      RCLCPP_FATAL(get_logger(), "Failed to load reference image");
      rclcpp::shutdown();
      return;
    }
    cv::GaussianBlur(ref_gray, ref_gray, cv::Size(7,7), 1.5);
    cv::threshold(ref_gray, ref_mask_, 128, 255, cv::THRESH_BINARY);

    // 3) Extract main contour from reference
    {
      std::vector<std::vector<cv::Point>> ref_contours;
      cv::findContours(ref_mask_, ref_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      if (ref_contours.empty()) {
        RCLCPP_FATAL(get_logger(), "No contours found in reference image");
        rclcpp::shutdown();
        return;
      }
      auto largest = std::max_element(
        ref_contours.begin(), ref_contours.end(),
        [](auto &a, auto &b){ return cv::contourArea(a) < cv::contourArea(b); }
      );
      ref_contour_ = *largest;
      RCLCPP_INFO(get_logger(),
        "Reference contour loaded (area=%.1f)", cv::contourArea(ref_contour_));
    }

    // 4) Set up blob detector (optional)
    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.minArea = 200.0f;
    params.maxArea = 50000.0f;
    blob_detector_ = cv::SimpleBlobDetector::create(params);

    // 5) Create display windows (optional)
    cv::namedWindow("Detection", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Mask",      cv::WINDOW_AUTOSIZE);
    cv::startWindowThread();

    // 6) Publisher for annotated images (now RELIABLE)
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "detection/image",
      rclcpp::QoS(10).reliable()
    );

    // 7) Subscribe to the color image topic
    RCLCPP_INFO(get_logger(), "Subscribing to /camera/color/image_raw");
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/infra1/image_rect_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&ObjectDetectionNode::imageCallback, this, std::placeholders::_1)
    );
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    // Convert ROS image to OpenCV BGR
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
      return;
    }
    if (frame.empty()) {
      RCLCPP_WARN(get_logger(), "Empty frame received");
      return;
    }

    // Preprocess: gray, blur, threshold
    cv::Mat gray, blurred, mask;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(7,7), 1.5);
    cv::threshold(blurred, mask, 128, 255, cv::THRESH_BINARY_INV);

    // 1) Shape matching: find only the best contour match
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_score = std::numeric_limits<double>::max();
    std::vector<cv::Point> best_contour;
    for (auto &cnt : contours) {
      double area = cv::contourArea(cnt);
      if (area < 500.0) continue;  // skip small noise
      double score = cv::matchShapes(cnt, ref_contour_, cv::CONTOURS_MATCH_I1, 0.0);
      if (score < best_score) {
        best_score = score;
        best_contour = cnt;
      }
    }

    // Draw the best match (if any)
    if (!best_contour.empty()) {
      std::vector<cv::Point> hull;
      cv::convexHull(best_contour, hull);
      cv::polylines(frame, hull, true, cv::Scalar(0,255,0), 2);
      RCLCPP_DEBUG(get_logger(), "Best match score: %.3f", best_score);
    }

    // 2) Blob detection (optional)
    std::vector<cv::KeyPoint> keypoints;
    blob_detector_->detect(mask, keypoints);
    cv::drawKeypoints(frame, keypoints, frame,
                      cv::Scalar(0,0,255),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // 3) Convert BGR -> RGB for RViz
    cv::Mat rgb_frame;
    cv::cvtColor(frame, rgb_frame, cv::COLOR_BGR2RGB);

    // 4) Publish annotated image as "rgb8"
    auto out_msg = cv_bridge::CvImage(msg->header, "rgb8", rgb_frame).toImageMsg();
    image_pub_->publish(*out_msg);

    // 5) Show windows (optional)
    cv::imshow("Mask",      mask);
    cv::imshow("Detection", frame);
    cv::waitKey(1);
  }

  // Members
  std::string ref_path_;
  cv::Mat ref_mask_;
  std::vector<cv::Point> ref_contour_;
  cv::Ptr<cv::SimpleBlobDetector> blob_detector_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    image_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
