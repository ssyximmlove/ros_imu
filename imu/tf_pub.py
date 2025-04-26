import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

class ImuTFPublisher(Node):
    def __init__(self):
        super().__init__('imu_tf_publisher')

        # 声明参数
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'imu_link')

        # 获取参数值
        self.imu_topic = self.get_parameter('imu_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        # 创建 TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 订阅 IMU 消息
        self.subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10)

        self.get_logger().info(f'IMU TF发布节点已启动，订阅话题: {self.imu_topic}')
        self.get_logger().info(f'发布TF: {self.parent_frame} -> {self.child_frame}')

    def imu_callback(self, msg):
        """处理IMU消息并发布TF变换"""
        t = TransformStamped()

        # 重新设置时间戳，不直接用IMU的
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # 位置（可以根据实际情况改）
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # 旋转，直接用IMU的 orientation
        t.transform.rotation = msg.orientation

        # 发送TF
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
