#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


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

		self.arm_state_sub = self.create_subscription(
			JointTrajectoryControllerState,
			'~/arm_position_commands',
			self.arm_state_callback,
			10,
		)

		self.gripper_state_sub = None
		if use_gripper:
			self.gripper_state_sub = self.create_subscription(
				JointTrajectoryControllerState,
				'/gripper_controller/state',
				self.gripper_state_callback,
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
			"Minimal JointStatePublisher initialized (joints=%d, use_gripper=%s)"
			% (len(self.joint_names), str(use_gripper).lower())
		)

	def _warn_throttle(self, key: str, message: str, period_sec: float = 2.0):
		now_ns = self.get_clock().now().nanoseconds
		period_ns = int(period_sec * 1e9)
		last_ns = self._last_warn_ns.get(key)
		if last_ns is None or (now_ns - last_ns) >= period_ns:
			self.get_logger().warn(message)
			self._last_warn_ns[key] = now_ns

	def _apply_controller_state(self, joint_names, positions, velocities, efforts):
		with self._state_lock:
			for index, joint_name in enumerate(joint_names):
				mapped_index = self.joint_name_to_index.get(joint_name)
				if mapped_index is None:
					continue

				if index < len(positions):
					self.joint_positions[mapped_index] = positions[index]
				if index < len(velocities):
					self.joint_velocities[mapped_index] = velocities[index]
				if index < len(efforts):
					self.joint_efforts[mapped_index] = efforts[index]

	def arm_state_callback(self, msg: JointTrajectoryControllerState):
		actual = msg.actual
		self._apply_controller_state(
			msg.joint_names,
			actual.positions,
			actual.velocities,
			actual.effort,
		)

	def gripper_state_callback(self, msg: JointTrajectoryControllerState):
		actual = msg.actual
		self._apply_controller_state(
			msg.joint_names,
			actual.positions,
			actual.velocities,
			actual.effort,
		)

	def arm_velocity_cmd_callback(self, msg: Float64MultiArray):
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
			if self.has_velocity_command and self.has_last_publish_time:
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
