#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class GripperTestNode : public rclcpp::Node
{
public:
  GripperTestNode() : Node("gripper_test_node")
  {
    // 创建夹爪轨迹发布器
    gripper_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/xarm_gripper_traj_controller/joint_trajectory", 10);

    // 创建服务，用于打开夹爪
    open_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/open_gripper",
      std::bind(&GripperTestNode::handleOpenGripper, this, std::placeholders::_1, std::placeholders::_2));

    // 创建服务，用于关闭夹爪
    close_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/close_gripper",
      std::bind(&GripperTestNode::handleCloseGripper, this, std::placeholders::_1, std::placeholders::_2));

    // 创建服务，用于设置指定的夹爪位置
    position_service_ = this->create_service<std_srvs::srv::SetBool>(
      "~/set_gripper_position",
      std::bind(&GripperTestNode::handleSetPosition, this, std::placeholders::_1, std::placeholders::_2));

    // 创建定时器，用于输出调试信息
    timer_ = this->create_wall_timer(2s, std::bind(&GripperTestNode::timerCallback, this));

    // 获取参数
    this->declare_parameter("gripper_joint_names", std::vector<std::string>{"drive_joint", "left_finger_joint", "right_finger_joint"});
    this->declare_parameter("open_position", 0.8);
    this->declare_parameter("close_position", 0.01);
    
    joint_names_ = this->get_parameter("gripper_joint_names").as_string_array();
    open_position_ = this->get_parameter("open_position").as_double();
    close_position_ = this->get_parameter("close_position").as_double();
    
    // 初始化last_command_time_为0，避免比较不同时间源的问题
    last_command_time_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "Gripper test node started!");
    RCLCPP_INFO(this->get_logger(), "Joint names: %zu joints configured", joint_names_.size());
    for (const auto& name : joint_names_) {
      RCLCPP_INFO(this->get_logger(), "- %s", name.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Open position: %.2f", open_position_);
    RCLCPP_INFO(this->get_logger(), "Close position: %.2f", close_position_);
    RCLCPP_INFO(this->get_logger(), "Available services:");
    RCLCPP_INFO(this->get_logger(), "- %s/open_gripper", this->get_name());
    RCLCPP_INFO(this->get_logger(), "- %s/close_gripper", this->get_name());
    RCLCPP_INFO(this->get_logger(), "- %s/set_gripper_position", this->get_name());
  }

private:
  // 处理打开夹爪请求
  void handleOpenGripper(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to open gripper");
    bool success = moveGripper(open_position_);
    response->success = success;
    response->message = success ? "Gripper opened successfully" : "Failed to open gripper";
  }

  // 处理关闭夹爪请求
  void handleCloseGripper(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to close gripper");
    bool success = moveGripper(close_position_);
    response->success = success;
    response->message = success ? "Gripper closed successfully" : "Failed to close gripper";
  }

  // 处理设置夹爪位置请求
  void handleSetPosition(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    // 这里我们使用data字段作为位置值的指示器 
    // true表示打开，false表示关闭
    double position = request->data ? open_position_ : close_position_;
    RCLCPP_INFO(this->get_logger(), "Received request to set gripper position: %.2f", position);
    bool success = moveGripper(position);
    response->success = success;
    response->message = success ? "Gripper position set successfully" : "Failed to set gripper position";
  }

  // 控制夹爪移动到指定位置
  bool moveGripper(double position)
  {
    RCLCPP_INFO(this->get_logger(), "Moving gripper to position: %.2f", position);
    
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = this->now();
    trajectory.joint_names = joint_names_;
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    // 为每个关节设置相同的位置
    point.positions.resize(joint_names_.size(), position);
    point.velocities.resize(joint_names_.size(), 0.1);  // 低速移动
    point.accelerations.resize(joint_names_.size(), 0.1);  // 低加速度
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);  // 1秒内完成
    
    trajectory.points.push_back(point);
    
    RCLCPP_INFO(this->get_logger(), "Publishing trajectory command with %zu joints", trajectory.joint_names.size());
    gripper_publisher_->publish(trajectory);
    
    // 记录最后一次发送命令的时间（使用double而不是rclcpp::Time）
    last_command_time_ = this->now().seconds();
    
    return true;
  }

  // 定时器回调，用于输出调试信息
  void timerCallback()
  {
    if (last_command_time_ > 0.0) {
      double elapsed = this->now().seconds() - last_command_time_;
      RCLCPP_DEBUG(this->get_logger(), "Time since last command: %.2f seconds", elapsed);
    }
  }

  // 发布器
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_publisher_;
  
  // 服务
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr position_service_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 夹爪配置
  std::vector<std::string> joint_names_;
  double open_position_;
  double close_position_;
  
  // 最后一次命令时间（使用double而不是rclcpp::Time避免时间源比较问题）
  double last_command_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}