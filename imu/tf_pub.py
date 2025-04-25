#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ImuTfPublisher(Node):
	def __init__(self):
		super().__init__('imu_tf_publisher')

		# 创建TF广播器
		self.tf_broadcaster = TransformBroadcaster(self)

		# 定义参数
		self.declare_parameter('imu_topic', 'imu/data')
		self.declare_parameter('parent_frame', 'base_link')
		self.declare_parameter('child_frame', 'imu_link')

		# 获取参数值
		self.imu_topic = self.get_parameter('imu_topic').value
		self.parent_frame = self.get_parameter('parent_frame').value
		self.child_frame = self.get_parameter('child_frame').value

		# 订阅IMU消息
		self.subscription = self.create_subscription(
			Imu,
			self.imu_topic,
			self.imu_callback,
			10)

		self.get_logger().info(f'IMU TF发布节点已启动，订阅话题: {self.imu_topic}')
		self.get_logger().info(f'发布TF: {self.parent_frame} -> {self.child_frame}')

	def imu_callback(self, msg):
		"""处理IMU消息并发布TF变换"""
		# 创建变换消息
		t = TransformStamped()

		# 设置头信息
		t.header = msg.header
		t.header.frame_id = self.parent_frame
		t.child_frame_id = self.child_frame

		# 设置IMU在机器人上的位置（根据实际情况调整）
		t.transform.translation.x = 0.0
		t.transform.translation.y = 0.0
		t.transform.translation.z = 0.0

		# 设置方向(直接使用IMU消息中的四元数)
		t.transform.rotation = msg.orientation

		# 发布变换
		self.tf_broadcaster.sendTransform(t)


def main(args=None):
	rclpy.init(args=args)
	node = ImuTfPublisher()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()