#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/opencv.hpp>

class VisionNode : public rclcpp::Node
{
public:
    VisionNode()
    : Node("vision_node")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("red_block_position", 10);

        sub_color_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/color/image_raw", 10,
            std::bind(&VisionNode::colorCallback, this, std::placeholders::_1)
        );

        sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/aligned_depth_to_color/image_raw", 10,
            std::bind(&VisionNode::depthCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "VisionNode started.");

        depth_ready_ = false;
    }

private:
    void colorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_ptr_color_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        processImages(msg->header.stamp);
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                cv_ptr_depth_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                cv_ptr_depth_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            } else {
                RCLCPP_WARN(this->get_logger(), "Unsupported depth image encoding: %s", msg->encoding.c_str());
                return;
            }
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        depth_ready_ = true;
        // 注意这里也需要传递时间戳，否则可能深度与彩色时间戳不一致
        // 这里可以不用传深度时间戳，因为点的位置是基于彩色图像的时间戳
    }

    void processImages(const rclcpp::Time &stamp)
    {
        if (!cv_ptr_color_ || !depth_ready_)
            return;

        cv::Mat img = cv_ptr_color_->image;

        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask1, mask2;
        cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), mask2);
        cv::Mat mask = mask1 | mask2;

        cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No red block detected");
            return;
        }

        size_t max_idx = 0;
        double max_area = 0;
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area > max_area) {
                max_area = area;
                max_idx = i;
            }
        }

        cv::Moments m = cv::moments(contours[max_idx]);
        if (m.m00 == 0) return;

        int cx = int(m.m10 / m.m00);
        int cy = int(m.m01 / m.m00);

        float depth = 0.0f;
        if (cv_ptr_depth_->image.type() == CV_16UC1) {
            uint16_t d = cv_ptr_depth_->image.at<uint16_t>(cy, cx);
            depth = d * 0.001f;
        } else if (cv_ptr_depth_->image.type() == CV_32FC1) {
            depth = cv_ptr_depth_->image.at<float>(cy, cx);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported depth image type");
            return;
        }

        if (depth <= 0.1 || depth > 2.0) {
            RCLCPP_WARN(this->get_logger(), "Depth value out of range: %.3f", depth);
            return;
        }

        const double fx = 640.51;
        const double fy = 640.51;
        const double cx_cam = 640.0;
        const double cy_cam = 360.0;


        double x = (cx - cx_cam) * depth / fx;
        double y = (cy - cy_cam) * depth / fy;
        double z = depth;

        geometry_msgs::msg::PointStamped pt_msg;
        pt_msg.header.stamp = stamp;  
        pt_msg.header.frame_id = "camera_color_optical_frame";
        pt_msg.point.x = x;
        pt_msg.point.y = y;
        pt_msg.point.z = z;

        pub_->publish(pt_msg);

        RCLCPP_INFO(this->get_logger(),
            "Red block detected at pixel (%d, %d), depth %.3f m, point (%.3f, %.3f, %.3f) in camera frame",
            cx, cy, depth, x, y, z);
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;

    cv_bridge::CvImagePtr cv_ptr_color_;
    cv_bridge::CvImagePtr cv_ptr_depth_;
    bool depth_ready_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
