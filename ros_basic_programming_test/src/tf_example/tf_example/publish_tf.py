import rclpy    #ros2python客户端
from rclpy.node import Node      # ros2节点基类，所有ros2节点都继承自这个类
import tf2_ros          #ros2中tf变化库
import geometry_msgs.msg    # ros2 几何形状消息类型
import time         

class TFPubNode(Node):
    def __init__(self):
        super().__init__('tf_pub_node')    # 节点名为tf_pub_node

        # 创建TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) # TransformBroadcaster对象,用于将TF变换广播到ROS网络中

        # 定时发布TF 1秒发布一次 
        self.timer = self.create_timer(1.0, self.publish_transform)
        # 初始化时间变量（以秒为单位）
        self.time_elapsed = 0.0
        # 初始化TF变换
        self.static_transform = geometry_msgs.msg.TransformStamped()
        self.static_transform.header.frame_id = 'world_link'    # 设置变换的参考坐标系（父坐标系）
        self.static_transform.child_frame_id = 'base_link'      # 设置目标坐标系（子坐标系）

        # 设置静态变换  这里是沿着x轴平移1米 
        # 方向角和四元数部分值为0的可以省略不写，默认值为0,此处写上为了自己看得懂
        self.static_transform.transform.translation.x = 1.0
        self.static_transform.transform.translation.y = 0.0
        self.static_transform.transform.translation.z = 0.0
        self.static_transform.transform.rotation.x = 0.0
        self.static_transform.transform.rotation.y = 0.0
        self.static_transform.transform.rotation.z = 0.0
        self.static_transform.transform.rotation.w = 1.0

        self.get_logger().info('TF发布节点已启动')

    def publish_transform(self):
        # 发布静态TF
        self.static_transform.header.stamp = self.get_clock().now().to_msg() # 更新时间戳
        self.tf_broadcaster.sendTransform(self.static_transform) # 发布world_link和base_link到ros网络

        # 发布动态TF
        # 同__init__
        self.time_elapsed += 0.1
        dynamic_transform = geometry_msgs.msg.TransformStamped()
        dynamic_transform.header.frame_id = 'base_link'
        dynamic_transform.child_frame_id = 'camera_link'
        dynamic_transform.transform.translation.x = self.time_elapsed
        dynamic_transform.transform.translation.y = 0.0
        dynamic_transform.transform.translation.z = 1.0
        dynamic_transform.transform.rotation.x = 0.0
        dynamic_transform.transform.rotation.y = 0.0
        dynamic_transform.transform.rotation.z = 0.0
        dynamic_transform.transform.rotation.w = 1.0

        dynamic_transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(dynamic_transform)

        self.get_logger().info(f'发布动态TF: {dynamic_transform.header.frame_id} -> {dynamic_transform.child_frame_id} 移动的x值: {dynamic_transform.transform.translation.x} 米')

def main(args=None):
    rclpy.init(args=args) 
    node = TFPubNode()  
    rclpy.spin(node)    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

