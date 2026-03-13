#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from piper_msgs.msg import PosCmd
from std_msgs.msg import Header

class PIPER:
    def __init__(self):
        self.node = None
        self.pub_joint = None
        self.joint_positions_received = False
        self.current_joint_positions = []
        self.target_joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def _ensure_node(self):
        if self.node is None:
            if not rclpy.ok():
                rclpy.init()
            self.node = Node('piper_control_node')
            self.pub_joint = self.node.create_publisher(JointState, '/joint_states', 10)
            self.arm_joint_state_publisher = self.pub_joint

    def joint_control_piper(self, j1, j2, j3, j4, j5, j6, gripper):
        self._ensure_node()
        joint_states_msgs = JointState()
        joint_states_msgs.header = Header()
        joint_states_msgs.header.stamp = self.node.get_clock().now().to_msg()
        joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
        joint_states_msgs.position = [j1, j2, j3, j4, j5, j6, gripper]
        self.pub_joint.publish(joint_states_msgs)
        print("send joint control piper command")

    def init_pose(self):
        self._ensure_node()
        target_joint_state = self.target_joint_state

        if self.joint_positions_received:
            current_positions = self.current_joint_positions
            self.node.get_logger().info(f"使用实际的当前关节位置: {current_positions}")

            duration = 0.5
            rate = 50
            steps = int(duration * rate)
            increments = [(target - current) / steps for current, target in zip(current_positions, target_joint_state)]

            rate_obj = self.node.create_rate(rate)
            start_time = self.node.get_clock().now()

            for step in range(steps + 1):
                interpolated_positions = [current + increment * step for current, increment in zip(current_positions, increments)]

                joint_states_msgs = JointState()
                joint_states_msgs.header = Header()
                joint_states_msgs.header.stamp = self.node.get_clock().now().to_msg()
                joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
                joint_states_msgs.position = interpolated_positions

                self.arm_joint_state_publisher.publish(joint_states_msgs)
                rate_obj.sleep()

            joint_states_msgs = JointState()
            joint_states_msgs.header = Header()
            joint_states_msgs.header.stamp = self.node.get_clock().now().to_msg()
            joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
            joint_states_msgs.position = target_joint_state
            self.arm_joint_state_publisher.publish(joint_states_msgs)

        else:
            start_time = self.node.get_clock().now()
            while (self.node.get_clock().now() - start_time).nanoseconds / 1e9 < 0.5:
                joint_states_msgs = JointState()
                joint_states_msgs.header = Header()
                joint_states_msgs.header.stamp = self.node.get_clock().now().to_msg()
                joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
                joint_states_msgs.position = target_joint_state
                self.arm_joint_state_publisher.publish(joint_states_msgs)
