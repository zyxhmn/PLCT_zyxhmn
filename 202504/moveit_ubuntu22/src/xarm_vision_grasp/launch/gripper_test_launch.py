from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xarm_vision_grasp',  # 替换为你的包名
            executable='gripper_test_node',
            name='gripper_test',
            output='screen',
            parameters=[
                {
                    # 设置你的夹爪关节名称
                    'gripper_joint_names': ['drive_joint', 'left_finger_joint', 'right_finger_joint'],
                    # 可以根据你的夹爪调整这些值
                    'open_position': 0.8,
                    'close_position': 0.01,
                }
            ]
        )
    ])