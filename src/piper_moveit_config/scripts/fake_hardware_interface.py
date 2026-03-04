#!/usr/bin/env python3
"""
Fake Hardware Interface for MoveIt2 Foxy
Simulates controller behavior for trajectory execution in simulation.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import threading
import time


class FakeHardwareInterface(Node):
    def __init__(self):
        super().__init__('fake_hardware_interface')
        
        # Current joint state
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
            'joint7', 'joint8'
        ]
        self.joint_positions = {name: 0.0 for name in self.joint_names}
        self.joint_velocities = {name: 0.0 for name in self.joint_names}
        self.lock = threading.Lock()
        
        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(
            JointState, 'joint_states', 10
        )
        
        # Create callback group for concurrent action handling
        self.callback_group = ReentrantCallbackGroup()
        
        # Action servers for trajectory execution
        self.arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/fake_arm_controller/follow_joint_trajectory',
            self.arm_execute_callback,
            callback_group=self.callback_group
        )
        
        self.gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/fake_gripper_controller/follow_joint_trajectory',
            self.gripper_execute_callback,
            callback_group=self.callback_group
        )
        
        # Timer to publish joint states
        self.create_timer(0.05, self.publish_joint_states, callback_group=self.callback_group)
        
        self.get_logger().info('Fake hardware interface started')
    
    def publish_joint_states(self):
        """Publish current joint state."""
        with self.lock:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [self.joint_positions[name] for name in self.joint_names]
            msg.velocity = [self.joint_velocities[name] for name in self.joint_names]
            self.joint_state_pub.publish(msg)
    
    def arm_execute_callback(self, goal_handle):
        return self.execute_trajectory(goal_handle, 'arm')

    def gripper_execute_callback(self, goal_handle):
        return self.execute_trajectory(goal_handle, 'gripper')
    
    def execute_trajectory(self, goal_handle, controller_type: str):
        trajectory = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()

        self.get_logger().info(
            f'Executing {controller_type} trajectory with {len(trajectory.points)} points'
        )

        try:
            start_time = time.time()

            goal_handle.succeed()

            for i, point in enumerate(trajectory.points):

                # Compute when this point should be reached
                desired_time = (
                    point.time_from_start.sec +
                    point.time_from_start.nanosec * 1e-9
                )

                # Wait until correct absolute time
                while (time.time() - start_time) < desired_time:
                    time.sleep(0.001)

                # Update joint states
                with self.lock:
                    for joint_name, position in zip(trajectory.joint_names, point.positions):
                        self.joint_positions[joint_name] = position

                    if point.velocities:
                        for joint_name, velocity in zip(trajectory.joint_names, point.velocities):
                            self.joint_velocities[joint_name] = velocity

            result.error_code = 0
            self.get_logger().info(
                f'{controller_type} trajectory executed successfully'
            )

        except Exception as e:
            self.get_logger().error(f'Error executing trajectory: {e}')
            goal_handle.abort()
            result.error_code = -1

        return result


def main(args=None):
    rclpy.init(args=args)
    interface = FakeHardwareInterface()
    
    # Use MultiThreadedExecutor to handle concurrent action calls
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(interface)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
