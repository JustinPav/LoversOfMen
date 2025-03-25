#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

cv::Point selectedPoint;
bool pointSelected = false;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (pointSelected)
    {
        cv::Vec3b color = cv_ptr->image.at<cv::Vec3b>(selectedPoint);
        int tolerance = 80;
        cv::Scalar lower(color[0] - tolerance, color[1] - tolerance, color[2] - tolerance);
        cv::Scalar upper(color[0] + tolerance, color[1] + tolerance, color[2] + tolerance);

        cv::Mat mask;
        cv::inRange(cv_ptr->image, lower, upper, mask);
        cv::imshow("Mask", mask);
    }
    cv::imshow("Camera Feed", cv_ptr->image);
    cv::waitKey(1);
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);

    if (pointSelected)
    {
        int index = selectedPoint.y * msg->width + selectedPoint.x;
        if (index < cloud.points.size())
        {
            pcl::PointXYZRGB point = cloud.points[index];
            ROS_INFO("Object Position: X=%.3f, Y=%.3f, Z=%.3f", point.x, point.y, point.z);
        }
        else
        {
            ROS_WARN("Selected point is out of range.");
        }
    }
}

void mouseCallback(int event, int x, int y, int, void *)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        selectedPoint = cv::Point(x, y);
        pointSelected = true;
        ROS_INFO("Selected Point: (%d, %d)", x, y);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_object_detection");
    ros::NodeHandle nh;

    cv::namedWindow("Camera Feed");
    cv::setMouseCallback("Camera Feed", mouseCallback);

    ros::Subscriber imageSub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, imageCallback);
    ros::Subscriber cloudSub = nh.subscribe("/camera/depth/color/points", 1, pointCloudCallback);

    ros::spin();
    return 0;
}
