from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Include the main xarm launch file
    xarm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('xarm_moveit_config'), 
                        'launch', 'xarm6_moveit_gazebo.launch.py')
        ]),
        launch_arguments={
            'add_realsense_d435i': 'true',
            'add_gripper': 'true'
        }.items()
    )
    
    # Add your arm control node
    arm_control_node = Node(
        package='xarm_vision_grasp',
        executable='arm_control_node',
        name='arm_control_node',
        output='screen'
    )
    
    return LaunchDescription([
        xarm_launch,
        arm_control_node
    ])