from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	imu_node = Node(
		package='imu',
		executable='main',
		name='imu_node',
		output='screen',
		parameters=[{
			'frame_id': 'imu_link'
		}],
	)

	tf_pub_node = Node(
		package='imu',
		executable='imu_tf_pub',
		name='imu_tf_pub_node',
		output='screen',
		parameters=[{
			'imu_topic': 'imu/data',
			'parent_frame': 'base_link',
			'child_frame': 'imu_link'
		}],
	)
	ld = LaunchDescription()
	ld.add_action(imu_node)
	ld.add_action(tf_pub_node)

	return ld

