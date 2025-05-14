#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
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

    // Extract its largest contour and compute reference metrics
    {
      std::vector<std::vector<cv::Point>> tmp;
      cv::findContours(ref_mask_, tmp, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      auto it = std::max_element(tmp.begin(), tmp.end(),
        [](auto &a, auto &b){ return cv::contourArea(a) < cv::contourArea(b); });
      ref_contour_ = *it;
      ref_area_ = cv::contourArea(ref_contour_);
      cv::Rect rrect = cv::boundingRect(ref_contour_);
      ref_aspect_ = static_cast<double>(rrect.width) / rrect.height;
      ref_aspect_min_ = ref_aspect_ * (1.0 - aspect_tol_);
      ref_aspect_max_ = ref_aspect_ * (1.0 + aspect_tol_);

      RCLCPP_INFO(get_logger(),
        "Loaded reference contour (area=%.1f, aspect=%.2f)",
        ref_area_, ref_aspect_);
    }

    // Show reference contour
    cv::Mat ref_vis;
    cv::cvtColor(ref_mask_, ref_vis, cv::COLOR_GRAY2BGR);
    cv::polylines(ref_vis, {ref_contour_}, true, {0,255,0}, 2);
    cv::namedWindow("Reference Contour", cv::WINDOW_AUTOSIZE);
    cv::imshow("Reference Contour", ref_vis);
    cv::waitKey(1);

    // Publishers & subscribers
    image_pub_  = create_publisher<sensor_msgs::msg::Image>("detection/image", 10);
    center_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("detection/center", 10);
    depth_pub_  = create_publisher<std_msgs::msg::Float32>("detection/depth", 10);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/infra1/image_rect_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&ObjectDetectionNode::imageCallback, this, std::placeholders::_1)
    );
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/depth/image_rect_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&ObjectDetectionNode::depthCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "Node initialized. Waiting for images...");
  }

private:
  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv::Mat depth;
    try {
      depth = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "Depth cv_bridge exception: %s", e.what());
      return;
    }
    std::lock_guard<std::mutex> lock(depth_mutex_);
    depth_frame_ = depth;
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "Color cv_bridge exception: %s", e.what());
      return;
    }
    if (frame.empty()) return;

    // Preprocess mask
    cv::Mat gray, blur, mask_adapt, mask;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, {7,7}, 1.5);
    cv::adaptiveThreshold(blur, mask_adapt, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY_INV, 11, 2);
    auto kernel = cv::getStructuringElement(cv::MORPH_RECT, {5,5});
    cv::morphologyEx(mask_adapt, mask, cv::MORPH_OPEN,  kernel, {}, 1);
    cv::morphologyEx(mask,       mask, cv::MORPH_CLOSE, kernel, {}, 1);

    // Find & score contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_score = std::numeric_limits<double>::infinity();
    std::vector<cv::Point> best_contour;
    double cx=0, cy=0;

    for (auto &c : contours) {
      double area = cv::contourArea(c);
      if (area < area_min_scale_*ref_area_ || area > area_max_scale_*ref_area_) continue;

      // Solidity filter
      std::vector<cv::Point> hull;
      cv::convexHull(c, hull);
      double hull_area = cv::contourArea(hull);
      double solidity = hull_area>0 ? area/hull_area : 0;
      if (solidity < solidity_threshold_) continue;

      // Aspect ratio filter
      cv::Rect br = cv::boundingRect(c);
      double ar = static_cast<double>(br.width)/br.height;
      if (ar < ref_aspect_min_ || ar > ref_aspect_max_) continue;

      // Approx & score
      std::vector<cv::Point> approx;
      cv::approxPolyDP(c, approx, 0.005*cv::arcLength(c,true), true);
      double score = cv::matchShapes(approx, ref_contour_, cv::CONTOURS_MATCH_I1, 0);

      // Debug draw
      cv::putText(frame, cv::format("%.2f", score), approx.front(),
                  cv::FONT_HERSHEY_PLAIN, 1, {255,0,0}, 1);
      cv::polylines(frame, approx, true, {255,0,0}, 1);

      if (score < best_score) {
        best_score = score;
        best_contour = approx;
        cv::Moments m = cv::moments(approx);
        cx = m.m10/m.m00;
        cy = m.m01/m.m00;
      }
    }

    // If found, extract depth and publish
    if (!best_contour.empty() && best_score <= shape_score_threshold_) {
      // Hull & center
      std::vector<cv::Point> hull;
      cv::convexHull(best_contour, hull);
      cv::polylines(frame, hull, true, {0,255,0}, 2);
      cv::circle(frame, {(int)cx,(int)cy}, 4, {255,0,0}, -1);

      // Depth lookup
      float depth_m = 0;
      {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        if (!depth_frame_.empty()) {
          if (depth_frame_.type()==CV_16U) {
            uint16_t d = depth_frame_.at<uint16_t>((int)cy,(int)cx);
            depth_m = d/1000.0f;
          } else if (depth_frame_.type()==CV_32F) {
            depth_m = depth_frame_.at<float>((int)cy,(int)cx);
          }
        }
      }

      RCLCPP_INFO(get_logger(), "Object center depth: %.3f m at pixel (%.0f, %.0f)", depth_m, cx, cy);

      // Publish center point
      geometry_msgs::msg::PointStamped pt;
      pt.header = msg->header;
      pt.point.x = cx;
      pt.point.y = cy;
      pt.point.z = depth_m;
      center_pub_->publish(pt);

      // Publish depth only
      std_msgs::msg::Float32 depth_msg;
      depth_msg.data = depth_m;
      depth_pub_->publish(depth_msg);

      // Overlay depth text
      cv::putText(frame, cv::format("Z=%.2fm", depth_m),
                  cv::Point((int)cx+5,(int)cy-5), cv::FONT_HERSHEY_PLAIN,
                  1.0, {0,255,0}, 1);
    }

    // Publish annotated image
    cv::Mat rgb;
    cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
    image_pub_->publish(*cv_bridge::CvImage(msg->header, "rgb8", rgb).toImageMsg());

    // Optional: visualize
    cv::imshow("Mask", mask);
    cv::imshow("Detection", frame);
    cv::waitKey(1);
  }

  // Members
  std::string ref_path_;
  double area_min_scale_, area_max_scale_, shape_score_threshold_;
  double solidity_threshold_, aspect_tol_;
  cv::Mat ref_mask_;
  std::vector<cv::Point> ref_contour_;
  double ref_area_, ref_aspect_, ref_aspect_min_, ref_aspect_max_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr center_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_pub_;

  // Depth buffer
  cv::Mat depth_frame_;
  std::mutex depth_mutex_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
