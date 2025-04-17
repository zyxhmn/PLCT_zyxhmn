#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>  
#include <memory>
#include <thread>
#include "xarm_vision_grasp/srv/grasp.hpp"

using Grasp = xarm_vision_grasp::srv::Grasp;
using namespace std::chrono_literals;

class ArmControlNode : public rclcpp::Node
{
  public:
    ArmControlNode()
    : Node("arm_control_node", rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true))
    {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      
      grasp_service_ = this->create_service<Grasp>(
        "grasp_service",
        std::bind(&ArmControlNode::handle_grasp_request, this, 
                  std::placeholders::_1, std::placeholders::_2));
                  
      RCLCPP_INFO(this->get_logger(), "Arm control node is ready to receive grasp requests");
    }
private:
  void handle_grasp_request(
    const std::shared_ptr<Grasp::Request> request,
    std::shared_ptr<Grasp::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received grasp request");
    RCLCPP_INFO(this->get_logger(), "视觉目标点位置 (frame_id=%s): x=%.4f, y=%.4f, z=%.4f",
            request->target_pose.header.frame_id.c_str(),
            request->target_pose.pose.position.x,
            request->target_pose.pose.position.y,
            request->target_pose.pose.position.z);


    try {
      auto move_group_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "xarm6");
      auto move_group_gripper = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "xarm_gripper");

      move_group_arm->setPlanningTime(10.0);
      move_group_arm->setMaxVelocityScalingFactor(0.5);
      move_group_arm->setMaxAccelerationScalingFactor(0.5);
      move_group_arm->setNumPlanningAttempts(10);
      move_group_arm->allowReplanning(true);

      geometry_msgs::msg::PoseStamped target_world;
      if (!tf_buffer_->canTransform("world", request->target_pose.header.frame_id,
                                    tf2::TimePointZero, 1s)) {
        response->success = false;
        response->message = "Unable to transform from camera frame to world frame";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      tf_buffer_->transform(request->target_pose, target_world, "world", tf2::durationFromSec(1.0));

      RCLCPP_INFO(this->get_logger(),
        "Target position in world frame: [%.3f, %.3f, %.3f]",
        target_world.pose.position.x,
        target_world.pose.position.y,
        target_world.pose.position.z);

      if (!moveGripper(move_group_gripper, 0.8)) {
        response->success = false;
        response->message = "Failed to open gripper";
        return;
      }

      geometry_msgs::msg::Pose pre_grasp_pose;
      pre_grasp_pose.position.x = target_world.pose.position.x;
      pre_grasp_pose.position.y = target_world.pose.position.y;
      pre_grasp_pose.position.z = target_world.pose.position.z + 0.1;
      pre_grasp_pose.orientation.x = 1;
      pre_grasp_pose.orientation.y = 0;
      pre_grasp_pose.orientation.z = 0;
      pre_grasp_pose.orientation.w = 0;

      if (!moveToTarget(move_group_arm, pre_grasp_pose)) {
        response->success = false;
        response->message = "Failed to move to pre-grasp position";
        return;
      }

      geometry_msgs::msg::Pose grasp_pose = pre_grasp_pose;
      grasp_pose.position.z = target_world.pose.position.z + 0.03;

      if (!moveToTarget(move_group_arm, grasp_pose)) {
        response->success = false;
        response->message = "Failed to move to grasp position";
        return;
      }

      if (!moveGripper(move_group_gripper, 0.01)) {
        response->success = false;
        response->message = "Failed to close gripper";
        return;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      geometry_msgs::msg::Pose lift_pose = grasp_pose;
      lift_pose.position.z += 0.1;

      if (!moveToTarget(move_group_arm, lift_pose)) {
        response->success = false;
        response->message = "Failed to lift object";
        return;
      }

      geometry_msgs::msg::Pose place_pose;
      place_pose.position.x = 0.3;
      place_pose.position.y = 0.0;
      place_pose.position.z = lift_pose.position.z;
      place_pose.orientation = lift_pose.orientation;

      if (!moveToTarget(move_group_arm, place_pose)) {
        response->success = false;
        response->message = "Failed to move to place position";
        return;
      }

      place_pose.position.z -= 0.1;

      if (!moveToTarget(move_group_arm, place_pose)) {
        response->success = false;
        response->message = "Failed to lower object";
        return;
      }

      if (!moveGripper(move_group_gripper, 0.8)) {
        response->success = false;
        response->message = "Failed to open gripper for release";
        return;
      }

      place_pose.position.z += 0.1;

      if (!moveToTarget(move_group_arm, place_pose)) {
        response->success = false;
        response->message = "Failed to move up after place";
        return;
      }

      move_group_arm->setNamedTarget("home");

      if (!executeArmPlan(move_group_arm)) {
        response->success = false;
        response->message = "Failed to return to home position";
        return;
      }

      response->success = true;
      response->message = "Successfully grasped and placed the object";
      RCLCPP_INFO(this->get_logger(), "Grasp operation completed successfully");

    } catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Exception during grasp operation: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }

  
  bool moveToTarget(
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group,
    const geometry_msgs::msg::Pose& target_pose)
  {
    move_group->setPoseTarget(target_pose);
    return executeArmPlan(move_group);
  }
  
  bool executeArmPlan(
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
      return (move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }
    return false;
  }
  
  bool moveGripper(
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& gripper_group,
    double position)
  {
    // For xArm gripper, we set the joint value for the gripper joints
    std::vector<double> joint_positions = {position, position};  // Set both finger joints
    gripper_group->setJointValueTarget(joint_positions);
    
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool success = (gripper_group->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
      return (gripper_group->execute(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }
    return false;
  }

  // TF-related components
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Service
  rclcpp::Service<Grasp>::SharedPtr grasp_service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // 创建一个多线程执行器
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<ArmControlNode>();
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}