#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TfTestNode : public rclcpp::Node
{
public:
    TfTestNode()
    : Node("tf_test_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TfTestNode::timerCallback, this)
        );
        RCLCPP_INFO(this->get_logger(), "TF Test Node started, checking transforms every 1 second.");
    }

private:
    void timerCallback()
    {
        printTransform("link_base", "camera_color_optical_frame");
        printTransform("link_base", "xarm_gripper_base_link");
        printTransform("link_base", "link_eef");
    }

    void printTransform(const std::string &target_frame, const std::string &source_frame)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_.lookupTransform(
                target_frame, source_frame, tf2::TimePointZero);
            auto &t = transformStamped.transform.translation;
            auto &r = transformStamped.transform.rotation;

            RCLCPP_INFO(this->get_logger(),
                "Transform from '%s' to '%s':\n  Translation: [%.3f, %.3f, %.3f]\n  Rotation (quat): [%.3f, %.3f, %.3f, %.3f]",
                target_frame.c_str(), source_frame.c_str(),
                t.x, t.y, t.z,
                r.x, r.y, r.z, r.w
            );
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could NOT transform '%s' to '%s': %s",
                source_frame.c_str(), target_frame.c_str(), ex.what());
        }
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
