#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <mutex>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
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
    declare_parameter<double>("shape_score_threshold", 0.02);
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

    if (ref_paths_.empty())
    {
      RCLCPP_FATAL(get_logger(), "No reference images provided. Set 'ref_image_paths' parameter.");
      rclcpp::shutdown();
      return;
    }

    // Assign labels F, A, I to first three refs, else R#
    for (size_t i = 0; i < ref_paths_.size(); ++i)
    {
      if (i == 0)
        //   ref_labels_.push_back("D");
        // else if (i == 1)
        ref_labels_.push_back("A");
      else if (i == 1)
        ref_labels_.push_back("I");
      else
        ref_labels_.push_back("R" + std::to_string(i));
    }

    // One publisher per label topic
    for (auto &lbl : ref_labels_)
    {
      label_pubs_.push_back(
          create_publisher<geometry_msgs::msg::PointStamped>(lbl, 10));
    }

    // Load & preprocess refs
    for (auto &path : ref_paths_)
    {
      cv::Mat gray = cv::imread(path, cv::IMREAD_GRAYSCALE);
      if (gray.empty())
      {
        RCLCPP_ERROR(get_logger(), "Failed to load '%s'", path.c_str());
        continue;
      }
      cv::GaussianBlur(gray, gray, {7, 7}, 1.5);
      cv::Mat mask;
      cv::threshold(gray, mask, 128, 255, cv::THRESH_BINARY_INV);
      std::vector<std::vector<cv::Point>> tmp;
      cv::findContours(mask, tmp, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      auto it = std::max_element(tmp.begin(), tmp.end(),
                                 [](auto &a, auto &b)
                                 { return cv::contourArea(a) < cv::contourArea(b); });
      if (it == tmp.end())
        continue;
      auto &cont = *it;

      ref_contours_.push_back(cont);
      double area = cv::contourArea(cont);
      ref_areas_.push_back(area);
      cv::Rect bb = cv::boundingRect(cont);
      double asp = double(bb.width) / bb.height;
      ref_aspect_mins_.push_back(asp * (1.0 - aspect_tol_));
      ref_aspect_maxs_.push_back(asp * (1.0 + aspect_tol_));
      ref_angles_.push_back(cv::minAreaRect(cont).angle);

      RCLCPP_INFO(get_logger(),
                  "Loaded '%s' (area=%.1f)", path.c_str(), area);
    }

    // Log file
    log_file_.open(log_file_path_);
    if (log_file_.is_open())
      log_file_ << "stamp,ref,x,y,z\n";
    else
      RCLCPP_WARN(get_logger(), "Cannot open log '%s'", log_file_path_.c_str());

    // ROS pubs/subs
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("detection/image", 10);
    center_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("detection/center", 10);
    world_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("detection/world_center", 10);
    depth_pub_ = create_publisher<std_msgs::msg::Float32>("detection/depth", 10);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/infra1/image_rect_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&ObjectDetectionNode::imageCallback, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/depth/image_rect_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&ObjectDetectionNode::depthCallback, this, std::placeholders::_1));
    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera/infra1/camera_info", 10,
        std::bind(&ObjectDetectionNode::camInfoCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Initialized with %zu refs.", ref_contours_.size());
  }

private:
  // --- camera-info
  void camInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &ci)
  {
    fx_ = ci->k[0];
    fy_ = ci->k[4];
    cx0_ = ci->k[2];
    cy0_ = ci->k[5];
    have_cam_info_ = true;
    cam_info_sub_.reset();
  }

  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    try
    {
      auto cvm = cv_bridge::toCvCopy(msg, msg->encoding)->image;
      std::lock_guard<std::mutex> lk(depth_mutex_);
      depth_frame_ = cvm;
    }
    catch (...)
    {
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    detected_centers_.clear();

    cv::Mat frame;
    try
    {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (...)
    {
      return;
    }
    if (frame.empty())
      return;

    // mask prep
    cv::Mat gray, blur, maskA, mask;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, {7, 7}, 1.5);
    cv::adaptiveThreshold(blur, maskA, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 11, 2);
    cv::morphologyEx(maskA, mask, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_RECT, {5, 5}));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, {5, 5}));

    // find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // --- NEW: contour-centric matching ---
    for (auto &c : contours)
    {
      double area = cv::contourArea(c);
      // score all refs
      std::vector<double> scores(ref_contours_.size(),
                                 std::numeric_limits<double>::infinity());

      for (size_t i = 0; i < ref_contours_.size(); ++i)
      {
        // area filter
        if (area < area_min_scale_ * ref_areas_[i] ||
            area > area_max_scale_ * ref_areas_[i])
          continue;
        // solidity
        std::vector<cv::Point> hull;
        cv::convexHull(c, hull);
        double sol = (cv::contourArea(hull) > 0)
                         ? area / cv::contourArea(hull)
                         : 0;
        if (sol < solidity_threshold_)
          continue;
        // aspect
        cv::Rect br = cv::boundingRect(c);
        double ar = double(br.width) / br.height;
        if (ar < ref_aspect_mins_[i] || ar > ref_aspect_maxs_[i])
          continue;
        // shape match
        std::vector<cv::Point> approx;
        cv::approxPolyDP(c, approx,
                         0.005 * cv::arcLength(c, true), true);
        scores[i] = cv::matchShapes(approx,
                                    ref_contours_[i], cv::CONTOURS_MATCH_I1, 0.0);
      }

      // pick best
      auto it = std::min_element(scores.begin(), scores.end());
      size_t best = std::distance(scores.begin(), it);
      double bestScore = *it;
      if (bestScore == std::numeric_limits<double>::infinity() ||
          bestScore > shape_score_threshold_)
        continue;

      // center & dedupe
      cv::Moments M = cv::moments(c);
      cv::Point2f ctr(M.m10 / M.m00, M.m01 / M.m00);
      bool dup = false;
      for (auto &p : detected_centers_)
        if (cv::norm(p - ctr) < dup_tol_px_)
        {
          dup = true;
          break;
        }
      if (dup)
        continue;
      detected_centers_.push_back(ctr);

      // depth
      float depth_m = 0;
      {
        std::lock_guard<std::mutex> lk(depth_mutex_);
        if (!depth_frame_.empty())
        {
          if (depth_frame_.type() == CV_16U)
            depth_m = depth_frame_.at<uint16_t>((int)ctr.y, (int)ctr.x) / 1000.0f;
          else if (depth_frame_.type() == CV_32F)
            depth_m = depth_frame_.at<float>((int)ctr.y, (int)ctr.x);
        }
      }
      std_msgs::msg::Float32 dm;
      dm.data = depth_m;
      depth_pub_->publish(dm);

      // project
      if (!have_cam_info_)
        continue;
      double Xc = (ctr.x - cx0_) * depth_m / fx_;
      double Yc = (ctr.y - cy0_) * depth_m / fy_;
      double Zc = depth_m;
      geometry_msgs::msg::PointStamped cam_pt;
      cam_pt.header = msg->header;
      cam_pt.point.x = Xc;
      cam_pt.point.y = Yc;
      cam_pt.point.z = Zc;
      center_pub_->publish(cam_pt);

      // TF→world + label topic
      try
      {
        auto w = tf_buffer_.transform(cam_pt, world_frame_,
                                      tf2::durationFromSec(0.1));
        world_pub_->publish(w);
        label_pubs_[best]->publish(w);
        if (log_file_.is_open())
          log_file_ << w.header.stamp.sec << "." << w.header.stamp.nanosec
                    << "," << best
                    << "," << w.point.x
                    << "," << w.point.y
                    << "," << w.point.z << "\n";
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_WARN(get_logger(), "TF: %s", ex.what());
      }

      // draw & annotate
      cv::convexHull(c, hull_);
      cv::polylines(frame, hull_, true, {0, 255, 0}, 2);
      cv::circle(frame, {(int)ctr.x, (int)ctr.y}, 4, {255, 0, 0}, -1);
      cv::putText(frame,
                  ref_labels_[best] + cv::format(" Z=%.2f", depth_m),
                  {int(ctr.x + 5), int(ctr.y - 5)},
                  cv::FONT_HERSHEY_PLAIN, 1.0, {0, 255, 0}, 1);
      cv::Point2f bpts[4];
      cv::minAreaRect(c).points(bpts);
      for (int k = 0; k < 4; ++k)
        cv::line(frame, bpts[k], bpts[(k + 1) % 4], {0, 255, 255}, 2);
    }

    // publish annotated image
    cv::Mat rgb;
    cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
    image_pub_->publish(
        *cv_bridge::CvImage(msg->header, "rgb8", rgb).toImageMsg());
  }

  // members…
  std::vector<std::string> ref_paths_, ref_labels_;
  double area_min_scale_, area_max_scale_, shape_score_threshold_;
  double solidity_threshold_, aspect_tol_;
  int dup_tol_px_;
  std::string world_frame_, log_file_path_;

  std::vector<std::vector<cv::Point>> ref_contours_;
  std::vector<double> ref_areas_, ref_aspect_mins_, ref_aspect_maxs_, ref_angles_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_, depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr center_pub_, world_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_pub_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr> label_pubs_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double fx_{0}, fy_{0}, cx0_{0}, cy0_{0};
  bool have_cam_info_{false};

  cv::Mat depth_frame_;
  std::mutex depth_mutex_;
  std::vector<cv::Point2f> detected_centers_;
  std::vector<cv::Point> hull_;
  std::ofstream log_file_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
