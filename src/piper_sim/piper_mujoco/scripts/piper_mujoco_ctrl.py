#!/usr/bin/env python3
# coding=utf-8

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
import os
import mujoco.viewer
import mujoco
from rclpy.node import Node
import rclpy
import sys
print(sys.executable)


class MujocoModel(Node):
    def __init__(self):
        super().__init__("mujoco_joint_controller")
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10)

        self.joint_targets = {}
        pkg_share_dir = get_package_share_directory('piper_description')
        model_path = os.path.join(
            pkg_share_dir,
            'mujoco_model',
            'piper_description.xml')
        model_path = os.path.abspath(model_path)

        self.get_logger().info(f"The model path is: {model_path}")

        # use the new mujoco python binding
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # use the passive viewer mode
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.timer = self.create_timer(0.01, self.control_loop)  # 100Hz control loop
        self.tolerance = 0.05  # angle error tolerance

    def joint_state_callback(self, msg):
        """ get joint angles from ROS 2 /joint_states topic """
        for i, name in enumerate(msg.name):
            self.joint_targets[name] = msg.position[i]

        # ensure joint8 is the negative of joint7
        if "joint7" in self.joint_targets:
            self.joint_targets["joint8"] = -self.joint_targets["joint7"]

    def pos_ctrl(self, joint_name, target_angle):
        """ control mujoco joint angle """
        # check if the joint exists
        joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
        )
        if joint_id == -1:
            self.get_logger().warn(
                f"Joint {joint_name} not found in Mujoco model."
            )
            return

        try:
            # get actuator ID
            actuator_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name
            )
            if actuator_id != -1:
                self.data.ctrl[actuator_id] = target_angle
        except Exception as e:
            self.get_logger().error(
                f"Error controlling joint {joint_name}: {e}"
            )

    def control_loop(self):
        """ control mujoco joint angle """
        if not self.viewer.is_running():
            self.get_logger().warn(
                "Viewer is closed. Shutting down node."
            )
            rclpy.shutdown()
            return

        for joint_name, target_angle in self.joint_targets.items():
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )
            if joint_id != -1:
                # get joint address in qpos
                qpos_addr = self.model.jnt_qposadr[joint_id]
                current_angle = self.data.qpos[qpos_addr]
                if abs(current_angle - target_angle) > self.tolerance:
                    self.pos_ctrl(joint_name, target_angle)

        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()

    def __del__(self):
        """ clean up resources """
        if hasattr(self, 'viewer') and self.viewer is not None:
            self.viewer.close()


def main():
    rclpy.init()
    mujoco_node = MujocoModel()
    # mujoco_node.declare_parameter('use_sim_time', True)
    try:
        rclpy.spin(mujoco_node)
    except KeyboardInterrupt:
        pass
    finally:
        mujoco_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
