#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory


class JointStatePublisher(Node):
	def __init__(self):
		super().__init__('minimal_joint_state_pub')

		self.declare_parameter('control_mode', 'position')
		self.declare_parameter('use_gripper', False)

		control_mode = self.get_parameter('control_mode').value
		use_gripper = bool(self.get_parameter('use_gripper').value)

		self.use_velocity_mode = control_mode == 'velocity'
		self.has_velocity_command = False
		self.has_last_publish_time = False
		self.last_publish_time = None

		self._state_lock = threading.Lock()
		self._last_warn_ns = {}

		if use_gripper:
			self.joint_names = [
				'joint1', 'joint2', 'joint3', 'joint4',
				'joint5', 'joint6', 'joint7', 'joint8'
			]
		else:
			self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

		self.joint_name_to_index = {name: index for index, name in enumerate(self.joint_names)}
		self.joint_positions = [0.0] * len(self.joint_names)
		self.joint_velocities = [0.0] * len(self.joint_names)
		self.joint_efforts = [0.0] * len(self.joint_names)

		self.joint_cmd_sub = self.create_subscription(
			JointTrajectory,
			'~/arm_position_commands',
			self.joint_cmd_callback,
			10,
		)

		self.arm_velocity_cmd_sub = self.create_subscription(
			Float64MultiArray,
			'~/arm_velocity_commands',
			self.arm_velocity_cmd_callback,
			10,
		)

		self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
		self.timer = self.create_timer(0.005, self.publish_joint_state)

		self.get_logger().info(
			"Minimal JointStatePublisher initialized (control_mode='%s', joints=%d)"
			% ('velocity' if self.use_velocity_mode else 'position', len(self.joint_names))
		)

	def _warn_throttle(self, key: str, message: str, period_sec: float = 2.0):
		now_ns = self.get_clock().now().nanoseconds
		period_ns = int(period_sec * 1e9)
		last_ns = self._last_warn_ns.get(key)
		if last_ns is None or (now_ns - last_ns) >= period_ns:
			self.get_logger().warn(message)
			self._last_warn_ns[key] = now_ns

	def joint_cmd_callback(self, msg: JointTrajectory):
		if self.use_velocity_mode:
			self._warn_throttle(
				'ignore_traj_in_velocity',
				'Ignoring JointTrajectory command while in velocity mode.',
			)
			return

		if not msg.points:
			self.get_logger().warn('Received empty trajectory')
			return

		point = msg.points[0]
		with self._state_lock:
			for index, joint_name in enumerate(msg.joint_names):
				mapped_index = self.joint_name_to_index.get(joint_name)
				if mapped_index is None:
					continue

				if index < len(point.positions):
					self.joint_positions[mapped_index] = point.positions[index]
				if index < len(point.velocities):
					self.joint_velocities[mapped_index] = point.velocities[index]
				if index < len(point.effort):
					self.joint_efforts[mapped_index] = point.effort[index]

	def arm_velocity_cmd_callback(self, msg: Float64MultiArray):
		if not self.use_velocity_mode:
			self._warn_throttle(
				'ignore_vel_in_position',
				'Ignoring Float64MultiArray velocity command while in position mode.',
			)
			return

		if not msg.data:
			self._warn_throttle(
				'empty_velocity_cmd',
				'Received empty arm velocity command.',
			)
			return

		with self._state_lock:
			joint_count = len(self.joint_names)
			copy_count = min(joint_count, len(msg.data))

			for index in range(copy_count):
				self.joint_velocities[index] = msg.data[index]

			for index in range(copy_count, joint_count):
				self.joint_velocities[index] = 0.0

			self.has_velocity_command = True

	def publish_joint_state(self):
		with self._state_lock:
			now = self.get_clock().now()
			if self.use_velocity_mode and self.has_velocity_command and self.has_last_publish_time:
				dt = (now - self.last_publish_time).nanoseconds / 1e9
				if dt > 0.0:
					for index in range(len(self.joint_positions)):
						self.joint_positions[index] += self.joint_velocities[index] * dt

			self.last_publish_time = now
			self.has_last_publish_time = True

			msg = JointState()
			msg.header.stamp = now.to_msg()
			msg.name = list(self.joint_names)
			msg.position = list(self.joint_positions)
			msg.velocity = list(self.joint_velocities)
			msg.effort = list(self.joint_efforts)
			self.joint_state_pub.publish(msg)


def main(args=None):
	rclpy.init(args=args)
	node = JointStatePublisher()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
