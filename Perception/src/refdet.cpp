#include <memory>
#include <string>
#include <vector>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ObjectDetectionNode : public rclcpp::Node {
public:
  ObjectDetectionNode()
  : Node("object_detection_node")
  {
    // --- PARAMETERS ---
    declare_parameter<std::string>("ref_image_path", "");
    declare_parameter<double>("area_min_scale", 0.001);
    declare_parameter<double>("area_max_scale", 2.0);
    declare_parameter<double>("shape_score_threshold", 1.0);
    declare_parameter<double>("solidity_threshold", 0.8);
    declare_parameter<double>("aspect_ratio_tolerance", 0.2);

    ref_path_ = get_parameter("ref_image_path").as_string();
    area_min_scale_ = get_parameter("area_min_scale").as_double();
    area_max_scale_ = get_parameter("area_max_scale").as_double();
    shape_score_threshold_ = get_parameter("shape_score_threshold").as_double();
    solidity_threshold_ = get_parameter("solidity_threshold").as_double();
    aspect_tol_ = get_parameter("aspect_ratio_tolerance").as_double();

    if (ref_path_.empty()) {
      RCLCPP_FATAL(get_logger(),
        "ref_image_path is empty. Pass with '-p ref_image_path:=/full/path.png'");
      rclcpp::shutdown();
      return;
    }

    // --- LOAD & PREPROCESS REFERENCE ---
    cv::Mat ref_gray = cv::imread(ref_path_, cv::IMREAD_GRAYSCALE);
    if (ref_gray.empty()) {
      RCLCPP_FATAL(get_logger(), "Failed to load reference '%s'", ref_path_.c_str());
      rclcpp::shutdown();
      return;
    }
    cv::GaussianBlur(ref_gray, ref_gray, {7,7}, 1.5);
    cv::threshold(ref_gray, ref_mask_, 128, 255, cv::THRESH_BINARY_INV);

    // Extract its largest contour
    {
      std::vector<std::vector<cv::Point>> tmp;
      cv::findContours(ref_mask_, tmp, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      if (tmp.empty()) {
        RCLCPP_FATAL(get_logger(), "No contour found in reference image");
        rclcpp::shutdown();
        return;
      }
      auto it = std::max_element(
        tmp.begin(), tmp.end(),
        [](auto &a, auto &b){ return cv::contourArea(a) < cv::contourArea(b); }
      );
      ref_contour_ = *it;
      ref_area_ = cv::contourArea(ref_contour_);
      
      // Reference aspect ratio
      cv::Rect rrect = cv::boundingRect(ref_contour_);
      ref_aspect_ = static_cast<double>(rrect.width) / rrect.height;
      ref_aspect_min_ = ref_aspect_ * (1.0 - aspect_tol_);
      ref_aspect_max_ = ref_aspect_ * (1.0 + aspect_tol_);

      RCLCPP_INFO(get_logger(),
        "Loaded reference contour (area=%.1f, aspect=%.2f)",
        ref_area_, ref_aspect_);
    }

    // --- VISUALIZE REFERENCE CONTOUR ---
    cv::Mat ref_vis;
    cv::cvtColor(ref_mask_, ref_vis, cv::COLOR_GRAY2BGR);
    cv::polylines(
      ref_vis,
      std::vector<std::vector<cv::Point>>{ref_contour_},
      true,
      cv::Scalar(0,255,0),
      2
    );
    cv::namedWindow("Reference Contour", cv::WINDOW_AUTOSIZE);
    cv::imshow("Reference Contour", ref_vis);
    cv::waitKey(1);

    // --- ROS 2 PUBLISHERS & SUBSCRIBER ---
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "detection/image", rclcpp::QoS(10).reliable());
    center_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "detection/center", rclcpp::QoS(10).reliable());

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/infra1/image_rect_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&ObjectDetectionNode::imageCallback, this,
                std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Node initialized. Waiting for images...");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    // A) Convert to OpenCV BGR
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    if (frame.empty()) {
      RCLCPP_WARN(get_logger(), "Empty frame");
      return;
    }

    // B) Preprocess to binary mask
    cv::Mat gray, blur, mask_adapt, mask;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, {7,7}, 1.5);
    cv::adaptiveThreshold(
      blur, mask_adapt, 255,
      cv::ADAPTIVE_THRESH_GAUSSIAN_C,
      cv::THRESH_BINARY_INV,
      11, 2
    );
    // Morphological open+close to remove noise
    auto kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(5,5)
    );
    cv::morphologyEx(mask_adapt, mask,
                     cv::MORPH_OPEN, kernel, cv::Point(-1,-1), 1);
    cv::morphologyEx(mask, mask,
                     cv::MORPH_CLOSE, kernel, cv::Point(-1,-1), 1);

    // C) Find contours & score them
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_score = std::numeric_limits<double>::infinity();
    std::vector<cv::Point> best_contour;

    for (auto &c : contours) {
      double area = cv::contourArea(c);
      if (area < area_min_scale_*ref_area_ ||
          area > area_max_scale_*ref_area_) continue;
      // solidity filter
      std::vector<cv::Point> hull;
      cv::convexHull(c, hull);
      double hull_area = cv::contourArea(hull);
      double solidity = (hull_area > 0) ? (area / hull_area) : 0;
      if (solidity < solidity_threshold_) continue;
      // aspect ratio filter
      cv::Rect br = cv::boundingRect(c);
      double ar = static_cast<double>(br.width) / br.height;
      if (ar < ref_aspect_min_ || ar > ref_aspect_max_) continue;

      // finer approx
      std::vector<cv::Point> approx;
      cv::approxPolyDP(c, approx,
                       0.005 * cv::arcLength(c, true), true);
      double score = cv::matchShapes(approx, ref_contour_, cv::CONTOURS_MATCH_I1, 0.0);

      // debug: show all candidates
      cv::putText(frame, cv::format("%.2f", score), approx.front(),
                  cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255,0,0), 1);
      cv::polylines(frame, approx, true, cv::Scalar(255,0,0), 1);

      if (score < best_score) {
        best_score = score;
        best_contour = approx;
      }
    }

    // D) Draw & publish only if below threshold
    if (!best_contour.empty() && best_score <= shape_score_threshold_) {
      std::vector<cv::Point> hull;
      cv::convexHull(best_contour, hull);
      cv::polylines(frame, hull, true, cv::Scalar(0,255,0), 2);
      cv::Moments m = cv::moments(best_contour);
      double cx = m.m10 / m.m00;
      double cy = m.m01 / m.m00;
      cv::circle(frame, {int(cx),int(cy)}, 4, cv::Scalar(255,0,0), -1);

      geometry_msgs::msg::PointStamped pt;
      pt.header = msg->header;
      pt.point.x = cx;
      pt.point.y = cy;
      pt.point.z = 0.0;
      center_pub_->publish(pt);
    }

    // E) Publish & visualize
    cv::Mat rgb;
    cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
    auto out = cv_bridge::CvImage(msg->header, "rgb8", rgb).toImageMsg();
    image_pub_->publish(*out);
    cv::imshow("Mask", mask);
    cv::imshow("Detection", frame);
    cv::waitKey(1);
  }

  // parameters
  std::string ref_path_;
  double area_min_scale_, area_max_scale_, shape_score_threshold_;
  double solidity_threshold_, aspect_tol_;

  // reference data
  cv::Mat ref_mask_;
  std::vector<cv::Point> ref_contour_;
  double ref_area_{0.0}, ref_aspect_{0.0}, ref_aspect_min_{0.0}, ref_aspect_max_{0.0};

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr center_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
