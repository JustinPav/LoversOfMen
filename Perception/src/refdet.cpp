#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

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

    // 2) Load & preprocess the reference image
    cv::Mat ref_gray = cv::imread(ref_path_, cv::IMREAD_GRAYSCALE);
    if (ref_gray.empty()) {
      RCLCPP_FATAL(get_logger(), "Failed to load reference image: %s", ref_path_.c_str());
      rclcpp::shutdown();
      return;
    }
    cv::GaussianBlur(ref_gray, ref_gray, cv::Size(7,7), 1.5);
    cv::threshold(ref_gray, ref_mask_, 128, 255, cv::THRESH_BINARY);

    // 3) Extract the main contour from the reference mask
    {
      std::vector<std::vector<cv::Point>> ref_contours;
      cv::findContours(ref_mask_, ref_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      if (ref_contours.empty()) {
        RCLCPP_FATAL(get_logger(), "No contours found in reference image");
        rclcpp::shutdown();
        return;
      }
      // pick the largest by area
      auto largest = std::max_element(
        ref_contours.begin(), ref_contours.end(),
        [](auto &a, auto &b){ return cv::contourArea(a) < cv::contourArea(b); }
      );
      ref_contour_ = *largest;
    }

    // 4) Set up blob detector
    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.minArea = 200.0f;
    params.maxArea = 50000.0f;
    params.filterByCircularity = false;
    params.filterByConvexity  = false;
    params.filterByInertia    = false;
    blob_detector_ = cv::SimpleBlobDetector::create(params);

    // 5) Create display windows
    cv::namedWindow("Detection", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Mask",      cv::WINDOW_AUTOSIZE);

    // 6) Subscribe to the color image topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/infra1/image_rect_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&ObjectDetectionNode::imageCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "ObjectDetectionNode initialized, reference = %s", ref_path_.c_str());
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    // Convert to OpenCV BGR
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

    // Preprocess: gray, blur, threshold
    cv::Mat gray, blurred, mask;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(7,7), 1.5);
    cv::threshold(blurred, mask, 128, 255, cv::THRESH_BINARY_INV);

    // 1) Contour‐based shape matching
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (auto &cnt : contours) {
      double area = cv::contourArea(cnt);
      if (area < 500.0) continue;

      double score = cv::matchShapes(cnt, ref_contour_, cv::CONTOURS_MATCH_I1, 0.0);
      if (score < shape_match_thresh_) {
        // draw matched contour
        std::vector<cv::Point> hull;
        cv::convexHull(cnt, hull);
        cv::polylines(frame, hull, true, cv::Scalar(0,255,0), 2);
      }
    }

    // 2) Blob detection
    std::vector<cv::KeyPoint> keypoints;
    blob_detector_->detect(mask, keypoints);
    cv::drawKeypoints(
      frame, keypoints, frame,
      cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    );

    // Display
    cv::imshow("Mask",      mask);
    cv::imshow("Detection", frame);
    cv::waitKey(1);
  }

  // Members
  std::string ref_path_;
  cv::Mat ref_mask_;
  std::vector<cv::Point> ref_contour_;
  const double shape_match_thresh_{0.1};  // tweak this as needed
  cv::Ptr<cv::SimpleBlobDetector> blob_detector_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}