#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.hpp>

class RedBlockDetector : public rclcpp::Node
{
public:
    RedBlockDetector()
    : Node("red_block_detector")
    {
        image_sub_ = image_transport::create_camera_subscription(
            this,
            "/color/image_raw",
            std::bind(&RedBlockDetector::imageCallback, this, std::placeholders::_1, std::placeholders::_2),
            "raw"
        );

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/aligned_depth_to_color/image_raw",
            10,
            std::bind(&RedBlockDetector::depthCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Red Block Detector Node Initialized.");
    }

private:
    image_transport::CameraSubscriber image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    cv::Mat latest_depth_;
    image_geometry::PinholeCameraModel cam_model_;

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            latest_depth_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
                       const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg)
    {
        cam_model_.fromCameraInfo(info_msg);

        cv::Mat image;
        try {
            image = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // 图像处理：提取红色区域
        cv::Mat hsv, mask;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

        // 红色有两个区间
        cv::Mat mask1, mask2;
        cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), mask2);
        mask = mask1 | mask2;

        // 形态学操作
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // 找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto &contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 200) {
                cv::Rect bound = cv::boundingRect(contour);
                cv::Point center(bound.x + bound.width / 2, bound.y + bound.height / 2);

                // 像素坐标输出
                RCLCPP_INFO(this->get_logger(), "Detected red block at pixel: (%d, %d)", center.x, center.y);

                // 深度坐标输出
                if (!latest_depth_.empty() && center.y < latest_depth_.rows && center.x < latest_depth_.cols) {
                    uint16_t depth = latest_depth_.at<uint16_t>(center);
                    if (depth != 0) {
                        // mm -> m
                        double z = depth * 0.001;
                        double x, y;
                        cv::Point3d ray = cam_model_.projectPixelTo3dRay(cv::Point2d(center));
                        x = ray.x * z;
                        y = ray.y * z;
                        RCLCPP_INFO(this->get_logger(), "Approx 3D TF coord: [%.3f, %.3f, %.3f]", x, y, z);

                    } else {
                        RCLCPP_WARN(this->get_logger(), "Depth at block is zero");
                    }
                }

                // 画出来
                cv::rectangle(image, bound, cv::Scalar(0, 255, 0), 2);
                cv::circle(image, center, 4, cv::Scalar(255, 0, 0), -1);
                cv::putText(image, "Red Block", cv::Point(bound.x, bound.y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            }
        }

        // 显示图像
        cv::imshow("Red Block Detection", image);
        cv::waitKey(1);
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RedBlockDetector>());
    rclcpp::shutdown();
    return 0;
}
