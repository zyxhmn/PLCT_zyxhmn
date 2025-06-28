#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> 
#include <geometry_msgs/msg/point_stamped.hpp>
#include "xarm_vision_grasp/srv/grasp.hpp"
#include <iostream>
#include <memory>
#include <thread>
#include <mutex>

using Grasp = xarm_vision_grasp::srv::Grasp;

class ControlNode : public rclcpp::Node
{
public:
    ControlNode()
    : Node("control_node", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}))
    {
        client_ = this->create_client<Grasp>("grasp_service");

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for grasp_service...");
        }
        RCLCPP_INFO(this->get_logger(), "grasp_service is available.");

        // 订阅视觉节点发布的 PointStamped
        sub_red_block_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "red_block_position", 10,
            std::bind(&ControlNode::redBlockCallback, this, std::placeholders::_1));

        std::thread([this]() { this->terminalLoop(); }).detach();
    }

private:
    void redBlockCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "收到红色方块点，frame_id: %s, x=%.3f y=%.3f z=%.3f",
        //     msg->header.frame_id.c_str(),
        //     msg->point.x, msg->point.y, msg->point.z);

        std::lock_guard<std::mutex> lock(mutex_);
        latest_point_ = *msg;
        has_point_ = true;
    }

    void terminalLoop()
    {
        while (rclcpp::ok()) {
            std::cout << "是否确认抓取红色方块？(y/n): ";
            std::string input;
            std::getline(std::cin, input);

            if (input != "y" && input != "Y") {
                std::cout << "放弃抓取，继续等待..." << std::endl;
                continue;
            }

            geometry_msgs::msg::PoseStamped target_pose;

            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!has_point_) {
                    std::cout << "没有收到红色方块位置，无法抓取" << std::endl;
                    continue;
                }
                // 构造PoseStamped，位置用点坐标，姿态赋默认四元数
                target_pose.header = latest_point_.header;
                target_pose.pose.position.x = latest_point_.point.x;
                target_pose.pose.position.y = latest_point_.point.y;
                target_pose.pose.position.z = latest_point_.point.z;
                target_pose.pose.orientation.x = 0.0;
                target_pose.pose.orientation.y = 0.0;
                target_pose.pose.orientation.z = 0.0;
                target_pose.pose.orientation.w = 1.0;
            }

            auto request = std::make_shared<Grasp::Request>();
            request->target_pose = target_pose;

            auto future_result = client_->async_send_request(
                request,
                std::bind(&ControlNode::handle_response, this, std::placeholders::_1)
            );
        }
    }

    void handle_response(rclcpp::Client<Grasp>::SharedFuture future)
    {
        auto response = future.get();
        std::cout << "抓取结果: " << (response->success ? "成功" : "失败")
                  << ", 信息: " << response->message << std::endl;
    }

    rclcpp::Client<Grasp>::SharedPtr client_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_red_block_;

    geometry_msgs::msg::PointStamped latest_point_;
    bool has_point_ = false;
    std::mutex mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}