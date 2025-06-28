import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg

class TFListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')

        # 创建TF监听器
        self.tf_buffer = tf2_ros.Buffer() # 缓存对象，存储tf变换信息
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)  # 接受变换信息存入buffer

        # 定时器，每1秒获取一次TF信息
        self.timer = self.create_timer(1.0, self.listen_transform)

    def listen_transform(self):
        try:
            # 获取动态TF /camera_link到/base_link
            # 缓冲区中查找变换关系，base_link:目标坐标系 camera_link:源坐标系
            transform = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())   
            # 打印信息
            self.get_logger().info(f'Transform from /camera_link to /base_link: {transform.transform}')
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'无法获取TF变换: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TFListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

