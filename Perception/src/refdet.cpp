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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

    // Load & preprocess each reference
    for (const auto &path : ref_paths_) {
      cv::Mat gray = cv::imread(path, cv::IMREAD_GRAYSCALE);
      if (gray.empty()) {
        RCLCPP_ERROR(get_logger(), "Failed to load reference '%s'", path.c_str());
        continue;
      }
      cv::GaussianBlur(gray, gray, {7,7}, 1.5);
      cv::Mat mask;
      cv::threshold(gray, mask, 128, 255, cv::THRESH_BINARY_INV);

      // find largest contour
      std::vector<std::vector<cv::Point>> tmp;
      cv::findContours(mask, tmp, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      auto it = std::max_element(tmp.begin(), tmp.end(), [](auto &a, auto &b){ return cv::contourArea(a) < cv::contourArea(b); });
      if (it == tmp.end()) continue;
      auto &cont = *it;

      // store contour and shape metrics
      ref_contours_.push_back(cont);
      double area = cv::contourArea(cont);
      ref_areas_.push_back(area);
      cv::Rect bb = cv::boundingRect(cont);
      double aspect = static_cast<double>(bb.width)/bb.height;
      ref_aspects_.push_back(aspect);
      ref_aspect_mins_.push_back(aspect * (1.0 - aspect_tol_));
      ref_aspect_maxs_.push_back(aspect * (1.0 + aspect_tol_));

      // store reference orientation
      cv::RotatedRect ref_rect = cv::minAreaRect(cont);
      ref_angles_.push_back(ref_rect.angle);

      RCLCPP_INFO(get_logger(), "Loaded '%s': area=%.1f, aspect=%.2f, angle=%.1f", path.c_str(), area, aspect, ref_rect.angle);
    }

    // Open log file
    log_file_.open(log_file_path_, std::ios::out);
    if (log_file_.is_open()) {
      log_file_ << "timestamp,ref_index,x,y,z" << std::endl;
    } else {
      RCLCPP_WARN(get_logger(), "Could not open log file '%s'", log_file_path_.c_str());
    }

    // Publishers & subscribers
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("detection/image", 10);
    center_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("detection/center", 10);
    world_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("detection/world_center", 10);
    depth_pub_ = create_publisher<std_msgs::msg::Float32>("detection/depth", 10);

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
  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    cv::Mat depth;
    try { depth = cv_bridge::toCvCopy(msg, msg->encoding)->image; }
    catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "Depth cv_bridge exception: %s", e.what());
      return;
    }
    std::lock_guard<std::mutex> lock(depth_mutex_);
    depth_frame_ = depth;
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    detected_centers_.clear();

    cv::Mat frame;
    try { frame = cv_bridge::toCvCopy(msg, "bgr8")->image; }
    catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "Color cv_bridge exception: %s", e.what());
      return;
    }
    if (frame.empty()) return;

    // preprocess mask
    cv::Mat gray, blur, mask_adapt, mask;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, {7,7}, 1.5);
    cv::adaptiveThreshold(blur, mask_adapt, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY_INV, 11, 2);
    auto kernel = cv::getStructuringElement(cv::MORPH_RECT, {5,5});
    cv::morphologyEx(mask_adapt, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < ref_contours_.size(); ++i) {
      double best_score = std::numeric_limits<double>::infinity();
      std::vector<cv::Point> best_contour;
      double cx=0, cy=0;

      for (auto &c : contours) {
        double area = cv::contourArea(c);
        if (area < area_min_scale_ * ref_areas_[i] || area > area_max_scale_ * ref_areas_[i])
          continue;

        std::vector<cv::Point> hull;
        cv::convexHull(c, hull);
        double solidity = hull.empty() ? 0.0 : area / cv::contourArea(hull);
        if (solidity < solidity_threshold_) continue;

        cv::Rect br = cv::boundingRect(c);
        double ar = static_cast<double>(br.width)/br.height;
        if (ar < ref_aspect_mins_[i] || ar > ref_aspect_maxs_[i]) continue;

        std::vector<cv::Point> approx;
        cv::approxPolyDP(c, approx, 0.005*cv::arcLength(c,true), true);
        double score = cv::matchShapes(approx, ref_contours_[i], cv::CONTOURS_MATCH_I1, 0);
        if (score < best_score) {
          best_score = score;
          best_contour = approx;
          cv::Moments m = cv::moments(approx);
          cx = m.m10/m.m00; cy = m.m01/m.m00;
        }
      }

      if (!best_contour.empty() && best_score <= shape_score_threshold_) {
        // dedupe
        cv::Point2f center(cx, cy);
        bool duplicate = false;
        for (auto &p : detected_centers_) {
          if (cv::norm(p - center) < dup_tol_px_) { duplicate = true; break; }
        }
        if (duplicate) continue;
        detected_centers_.push_back(center);

        // publish camera-frame center
        geometry_msgs::msg::PointStamped cam_pt;
        cam_pt.header = msg->header;
        cam_pt.point.x = cx; cam_pt.point.y = cy; cam_pt.point.z = 0.0;
        center_pub_->publish(cam_pt);

        // get depth
        float depth_m = 0.0f;
        {
          std::lock_guard<std::mutex> lock(depth_mutex_);
          if (!depth_frame_.empty()) {
            if (depth_frame_.type()==CV_16U) depth_m = depth_frame_.at<uint16_t>((int)cy,(int)cx)/1000.0f;
            else if (depth_frame_.type()==CV_32F) depth_m = depth_frame_.at<float>((int)cy,(int)cx);
          }
        }
        std_msgs::msg::Float32 dmsg; dmsg.data = depth_m;
        depth_pub_->publish(dmsg);

        // transform to world frame & log
        try {
          cam_pt.point.z = depth_m;
          auto world_pt = tf_buffer_.transform(cam_pt, world_frame_, tf2::durationFromSec(0.1));
          world_pub_->publish(world_pt);
          if (log_file_.is_open()) {
            log_file_ << world_pt.header.stamp.sec << "." << world_pt.header.stamp.nanosec
                      << "," << i
                      << "," << world_pt.point.x
                      << "," << world_pt.point.y
                      << "," << world_pt.point.z
                      << std::endl;
          }
        } catch (tf2::TransformException &ex) {
          RCLCPP_WARN(get_logger(), "TF failure: %s", ex.what());
        }

        // compute orientation difference
        cv::RotatedRect det_rect = cv::minAreaRect(best_contour);
        double det_angle = det_rect.angle;
        double ref_angle = ref_angles_[i];
        double angle_diff = det_angle - ref_angle;
        if (angle_diff < -90.0) angle_diff += 180.0;
        else if (angle_diff > 90.0) angle_diff -= 180.0;

        // draw detection hull
        cv::convexHull(best_contour, hull_);
        cv::polylines(frame, hull_, true,                  {0,255,0}, 2);
        cv::circle(frame, {(int)cx,(int)cy}, 4,          {255,0,0}, -1);
        cv::putText(frame, cv::format("Z=%.2fm", depth_m),{int(cx+5), int(cy-5)},
                    cv::FONT_HERSHEY_PLAIN, 1.0,        {0,255,0}, 1);

        // draw rotated box & orientation
        cv::Point2f box_pts[4]; det_rect.points(box_pts);
        for (int k=0; k<4; ++k)
          cv::line(frame, box_pts[k], box_pts[(k+1)%4], {0,255,255}, 2);
        cv::putText(frame,
                    cv::format("Δθ=%.1f°", angle_diff),
                    {int(cx+5), int(cy+20)},
                    cv::FONT_HERSHEY_PLAIN,
                    1.0,
                    {0,255,255},
                    2);
      }
    }

    // publish annotated image
    cv::Mat rgb;
    cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
    image_pub_->publish(*cv_bridge::CvImage(msg->header, "rgb8", rgb).toImageMsg());
  }

  // parameters
  std::vector<std::string> ref_paths_;
  double area_min_scale_, area_max_scale_, shape_score_threshold_;
  double solidity_threshold_, aspect_tol_;
  int dup_tol_px_;
  std::string world_frame_, log_file_path_;

  // loaded refs
  std::vector<std::vector<cv::Point>> ref_contours_;
  std::vector<double> ref_areas_, ref_aspects_, ref_aspect_mins_, ref_aspect_maxs_, ref_angles_;

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_, depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr center_pub_, world_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_pub_;

  // TF2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // depth buffer
  cv::Mat depth_frame_;
  std::mutex depth_mutex_;

  // dedup storage per frame
  std::vector<cv::Point2f> detected_centers_;

  // log file
  std::ofstream log_file_;

  // drawing temp
  std::vector<cv::Point> hull_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
