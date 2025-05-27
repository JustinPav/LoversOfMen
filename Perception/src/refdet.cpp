#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <mutex>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ObjectDetectionNode : public rclcpp::Node
{
public:
  ObjectDetectionNode()
      : Node("object_detection_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
  {
    // --- PARAMETERS ---
    declare_parameter<std::vector<std::string>>("ref_image_paths", std::vector<std::string>());
    declare_parameter<double>("area_min_scale", 0.001);
    declare_parameter<double>("area_max_scale", 2.0);
    declare_parameter<double>("shape_score_threshold", 0.1);
    declare_parameter<double>("solidity_threshold", 0.8);
    declare_parameter<double>("aspect_ratio_tolerance", 0.2);
    declare_parameter<int>("duplicate_pixel_tolerance", 20);
    declare_parameter<std::string>("world_frame", "world");
    declare_parameter<std::string>("log_file_path", "/tmp/detections.csv");

    get_parameter("ref_image_paths", ref_paths_);
    get_parameter("area_min_scale", area_min_scale_);
    get_parameter("area_max_scale", area_max_scale_);
    get_parameter("shape_score_threshold", shape_score_threshold_);
    get_parameter("solidity_threshold", solidity_threshold_);
    get_parameter("aspect_ratio_tolerance", aspect_tol_);
    get_parameter("duplicate_pixel_tolerance", dup_tol_px_);
    get_parameter("world_frame", world_frame_);
    get_parameter("log_file_path", log_file_path_);

    if (ref_paths_.empty()) {
      RCLCPP_FATAL(get_logger(), "No reference images provided. Set 'ref_image_paths' parameter.");
      rclcpp::shutdown();
      return;
    }

    // --- LOAD & PROCESS REFERENCES ---
    for (const auto &path : ref_paths_) {
      cv::Mat gray = cv::imread(path, cv::IMREAD_GRAYSCALE);
      if (gray.empty()) {
        RCLCPP_ERROR(get_logger(), "Failed to load reference '%s'", path.c_str());
        continue;
      }
      cv::GaussianBlur(gray, gray, {7,7}, 1.5);
      cv::Mat mask;
      cv::threshold(gray, mask, 128, 255, cv::THRESH_BINARY_INV);

      std::vector<std::vector<cv::Point>> tmp;
      cv::findContours(mask, tmp, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      auto it = std::max_element(tmp.begin(), tmp.end(),
                                 [](auto &a, auto &b){ return cv::contourArea(a) < cv::contourArea(b); });
      if (it == tmp.end()) continue;

      // store contour + metrics
      ref_contours_.push_back(*it);
      double area = cv::contourArea(*it);
      ref_areas_.push_back(area);
      cv::Rect bb = cv::boundingRect(*it);
      double aspect = static_cast<double>(bb.width)/bb.height;
      ref_aspects_.push_back(aspect);
      ref_aspect_mins_.push_back(aspect * (1.0 - aspect_tol_));
      ref_aspect_maxs_.push_back(aspect * (1.0 + aspect_tol_));

      cv::RotatedRect ref_rect = cv::minAreaRect(*it);
      ref_angles_.push_back(ref_rect.angle);

      RCLCPP_INFO(get_logger(), "Loaded '%s': area=%.1f, aspect=%.2f, angle=%.1f",
                  path.c_str(), area, aspect, ref_rect.angle);
    }

    // --- ASSIGN LABELS & CREATE LABEL-PUBLISHERS ---
    for (size_t i = 0; i < ref_contours_.size(); ++i) {
      std::string lbl;
      if      (i == 0) lbl = "F";
      else if (i == 1) lbl = "A";
      else if (i == 2) lbl = "I";
      else             lbl = "R" + std::to_string(i);
      ref_labels_.push_back(lbl);

      // publisher named exactly as the label
      label_pubs_.push_back(
        create_publisher<geometry_msgs::msg::PointStamped>(lbl, 10)
      );
    }

    // Open log file
    log_file_.open(log_file_path_, std::ios::out);
    if (log_file_.is_open()) {
      log_file_ << "timestamp,ref_label,x,y,z" << std::endl;
    } else {
      RCLCPP_WARN(get_logger(), "Could not open log file '%s'", log_file_path_.c_str());
    }

    // Standard publishers & subscribers
    image_pub_  = create_publisher<sensor_msgs::msg::Image>("detection/image", 10);
    center_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("detection/center", 10);
    world_pub_  = create_publisher<geometry_msgs::msg::PointStamped>("detection/world_center", 10);
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

    RCLCPP_INFO(get_logger(), "Node initialized with %zu references. Waiting for images...", ref_contours_.size());
  }

private:
  // ... [depthCallback unchanged] ...

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    detected_centers_.clear();
    cv::Mat frame;
    try { frame = cv_bridge::toCvCopy(msg, "bgr8")->image; }
    catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "Color cv_bridge exception: %s", e.what());
      return;
    }
    if (frame.empty()) return;

    // [mask generation and contour finding unchanged]

    for (size_t i = 0; i < ref_contours_.size(); ++i) {
      // [matching logic unchanged]

      if (!best_contour.empty() && best_score <= shape_score_threshold_) {
        // --- PUBLISH CAMERA-FRAME CENTER, DEPTH AS BEFORE ---
        geometry_msgs::msg::PointStamped cam_pt;
        cam_pt.header = msg->header;
        cam_pt.point.x = cx; cam_pt.point.y = cy; cam_pt.point.z = 0.0;
        center_pub_->publish(cam_pt);

        float depth_m = /* ... extract from depth_frame_ ... */;
        std_msgs::msg::Float32 dmsg; dmsg.data = depth_m;
        depth_pub_->publish(dmsg);

        // --- TRANSFORM TO WORLD, LOG & PUBLISH ---
        try {
          cam_pt.point.z = depth_m;
          auto world_pt = tf_buffer_.transform(cam_pt, world_frame_, tf2::durationFromSec(0.1));
          world_pub_->publish(world_pt);

          // also publish on the per-label topic:
          label_pubs_[i]->publish(world_pt);

          if (log_file_.is_open()) {
            log_file_ << world_pt.header.stamp.sec << "." << world_pt.header.stamp.nanosec
                      << "," << ref_labels_[i]
                      << "," << world_pt.point.x
                      << "," << world_pt.point.y
                      << "," << world_pt.point.z
                      << std::endl;
          }
        } catch (tf2::TransformException &ex) {
          RCLCPP_WARN(get_logger(), "TF failure: %s", ex.what());
        }

        // [drawing code unchanged]
      }
    }

    // publish annotated image unchanged...
  }

  // parameters & refs
  std::vector<std::string> ref_paths_;
  double area_min_scale_, area_max_scale_, shape_score_threshold_;
  double solidity_threshold_, aspect_tol_;
  int dup_tol_px_;
  std::string world_frame_, log_file_path_;

  // loaded references
  std::vector<std::vector<cv::Point>> ref_contours_;
  std::vector<double> ref_areas_, ref_aspects_, ref_aspect_mins_, ref_aspect_maxs_, ref_angles_;

  // new: labels & per-label publishers
  std::vector<std::string> ref_labels_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr> label_pubs_;

  // standard ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_, depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr center_pub_, world_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_pub_;

  // TF2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // depth buffer & drawing
  cv::Mat depth_frame_;
  std::mutex depth_mutex_;
  std::vector<cv::Point2f> detected_centers_;
  std::ofstream log_file_;
  std::vector<cv::Point> hull_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
