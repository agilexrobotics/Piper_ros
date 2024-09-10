#!/usr/bin/env python3
# -*-coding:utf8-*-
# 本文件为控制单个机械臂节点，控制夹爪机械臂运动
from typing import (
    Optional,
)
import rospy
import rosnode
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import threading
import argparse
from piper_sdk import *
from piper_sdk import C_PiperInterface
from piper_msgs.msg import PiperStatusMsg, PosCmd
from piper_msgs.srv import Enable, EnableResponse
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler  # 用于欧拉角到四元数的转换

def check_ros_master():
    try:
        rosnode.rosnode_ping('rosout', max_count=1, verbose=False)
        rospy.loginfo("ROS Master is running.")
    except rosnode.ROSNodeIOException:
        rospy.logerr("ROS Master is not running.")
        raise RuntimeError("ROS Master is not running.")

class C_PiperRosNode():
    """机械臂ros节点
    """
    def __init__(self) -> None:
        check_ros_master()
        rospy.init_node('piper_ctrl_single_node', anonymous=True)
        # 外部param参数
        # can路由名称
        self.can_port = "can0"
        if rospy.has_param('~can_port'):
            self.can_port = rospy.get_param("~can_port")
            rospy.loginfo("%s is %s", rospy.resolve_name('~can_port'), self.can_port)
        else: 
            rospy.loginfo("未找到can_port参数")
            exit(0)
        # 是否自动使能，默认不自动使能
        self.auto_enable = False
        if rospy.has_param('~auto_enable'):
            if(rospy.get_param("~auto_enable")):
                self.auto_enable = True
        rospy.loginfo("%s is %s", rospy.resolve_name('~auto_enable'), self.auto_enable)
        # 是否有夹爪，默认为有
        self.girpper_exist = True
        if rospy.has_param('~girpper_exist'):
            if(not rospy.get_param("~girpper_exist")):
                self.girpper_exist = False
        rospy.loginfo("%s is %s", rospy.resolve_name('~girpper_exist'), self.girpper_exist)
        # 是否是打开了rviz控制，默认为不是，如果打开了，gripper订阅的joint7关节消息会乘2倍
        self.rviz_ctrl_flag = False
        if rospy.has_param('~rviz_ctrl_flag'):
            if(rospy.get_param("~rviz_ctrl_flag")):
                self.rviz_ctrl_flag = True
        rospy.loginfo("%s is %s", rospy.resolve_name('~rviz_ctrl_flag'), self.rviz_ctrl_flag)

        self.joint_pub = rospy.Publisher('joint_states_single', JointState, queue_size=1)
        self.arm_status_pub = rospy.Publisher('arm_status', PiperStatusMsg, queue_size=1)
        self.end_pose_pub = rospy.Publisher('end_pose', Pose, queue_size=1)
        self.enable_service = rospy.Service('enable_srv', Enable, self.handle_enable_service)  # 创建服务
        
        self.__enable_flag = False
        # joint
        self.joint_states = JointState()
        self.joint_states.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_states.position = [0.0] * 7
        self.joint_states.velocity = [0.0] * 6
        self.joint_states.effort = [0.0] * 7
        
        # 创建piper类并打开can接口
        self.piper = C_PiperInterface(can_name=self.can_port)
        self.piper.ConnectPort()

        # 启动订阅线程
        sub_pos_th = threading.Thread(target=self.SubPosThread)
        sub_pos_th.daemon = True
        sub_pos_th.start()
        sub_joint_th = threading.Thread(target=self.SubJointThread)
        sub_enable_th = threading.Thread(target=self.SubEnableThread)
        
        sub_joint_th.daemon = True
        sub_enable_th.daemon = True
        
        sub_joint_th.start()
        sub_enable_th.start()

    def GetEnableFlag(self):
        return self.__enable_flag

    def Pubilsh(self):
        """机械臂消息发布
        """
        rate = rospy.Rate(200)  # 200 Hz
        enable_flag = False
        # 设置超时时间（秒）
        timeout = 5
        # 记录进入循环前的时间
        start_time = time.time()
        elapsed_time_flag = False
        while not rospy.is_shutdown():
            # print(self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status)
            if(self.auto_enable):
                while not (enable_flag):
                    elapsed_time = time.time() - start_time
                    print("--------------------")
                    enable_flag = self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
                    print("使能状态:",enable_flag)
                    self.piper.EnableArm(7)
                    self.piper.GripperCtrl(0,1000,0x01, 0)
                    if(enable_flag):
                        self.__enable_flag = True
                    print("--------------------")
                    # 检查是否超过超时时间
                    if elapsed_time > timeout:
                        print("超时....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(1)
                    pass
            if(elapsed_time_flag):
                print("程序自动使能超时,退出程序")
                exit(0)
            # 发布消息
            self.PublishArmState()
            self.PublishArmEndPose()
            self.PublishArmJointAndGripper()
            rate.sleep()

    def PublishArmState(self):
        # 机械臂状态
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        arm_status.arm_status = self.piper.GetArmStatus().arm_status.arm_status
        arm_status.mode_feedback = self.piper.GetArmStatus().arm_status.mode_feed
        arm_status.teach_status = self.piper.GetArmStatus().arm_status.teach_status
        arm_status.motion_status = self.piper.GetArmStatus().arm_status.motion_status
        arm_status.trajectory_num = self.piper.GetArmStatus().arm_status.trajectory_num
        arm_status.err_code = self.piper.GetArmStatus().arm_status.err_code
        arm_status.joint_1_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_1_angle_limit
        arm_status.joint_2_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_2_angle_limit
        arm_status.joint_3_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_3_angle_limit
        arm_status.joint_4_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_4_angle_limit
        arm_status.joint_5_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_5_angle_limit
        arm_status.joint_6_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_6_angle_limit
        arm_status.communication_status_joint_1 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_1
        arm_status.communication_status_joint_2 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_2
        arm_status.communication_status_joint_3 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_3
        arm_status.communication_status_joint_4 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_4
        arm_status.communication_status_joint_5 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_5
        arm_status.communication_status_joint_6 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_6
        self.arm_status_pub.publish(arm_status)
        
    def PublishArmJointAndGripper(self):
        # 机械臂关节角和夹爪位置
        # 由于获取的原始数据是度为单位扩大了1000倍，因此要转为弧度需要先除以1000，再乘3.14/180，然后限制小数点位数为5位
        joint_0:float = (self.piper.GetArmJointMsgs().joint_state.joint_1/1000) * 0.017444
        joint_1:float = (self.piper.GetArmJointMsgs().joint_state.joint_2/1000) * 0.017444
        joint_2:float = (self.piper.GetArmJointMsgs().joint_state.joint_3/1000) * 0.017444
        joint_3:float = (self.piper.GetArmJointMsgs().joint_state.joint_4/1000) * 0.017444
        joint_4:float = (self.piper.GetArmJointMsgs().joint_state.joint_5/1000) * 0.017444
        joint_5:float = (self.piper.GetArmJointMsgs().joint_state.joint_6/1000) * 0.017444
        joint_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle/1000000
        vel_0:float = self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed/1000
        vel_1:float = self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed/1000
        vel_2:float = self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed/1000
        vel_3:float = self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed/1000
        vel_4:float = self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed/1000
        vel_5:float = self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed/1000
        effort_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort/1000
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.position = [joint_0,joint_1, joint_2, joint_3, joint_4, joint_5,joint_6]  # Example values
        self.joint_states.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5]  # Example values
        self.joint_states.effort = [0, 0, 0, 0, 0, 0, effort_6]
        # 发布所有消息
        self.joint_pub.publish(self.joint_states)
    
    def PublishArmEndPose(self):
        # 末端位姿
        endpos = Pose()
        endpos.position.x = self.piper.ArmEndPose.end_pose.X_axis/1000000
        endpos.position.y = self.piper.ArmEndPose.end_pose.Y_axis/1000000
        endpos.position.z = self.piper.ArmEndPose.end_pose.Z_axis/1000000
        roll = self.piper.ArmEndPose.end_pose.RX_axis/1000
        pitch = self.piper.ArmEndPose.end_pose.RY_axis/1000
        yaw = self.piper.ArmEndPose.end_pose.RZ_axis/1000
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        endpos.orientation.x = quaternion[0]
        endpos.orientation.y = quaternion[1]
        endpos.orientation.z = quaternion[2]
        endpos.orientation.w = quaternion[3]
        self.end_pose_pub.publish(endpos)
    
    def SubPosThread(self):
        """机械臂末端位姿订阅
        
        """
        rospy.Subscriber('pos_cmd', PosCmd, self.pos_callback)
        rospy.spin()
    
    def SubJointThread(self):
        """机械臂关节订阅
        
        """
        rospy.Subscriber('joint_ctrl_single', JointState, self.joint_callback)
        rospy.spin()
    
    def SubEnableThread(self):
        """机械臂使能
        
        """
        rospy.Subscriber('enable_flag', Bool, self.enable_callback)
        rospy.spin()

    def pos_callback(self, pos_data):
        """机械臂末端位姿订阅回调函数

        Args:
            pos_data (): 
        """
        rospy.loginfo("Received PosCmd:")
        rospy.loginfo("x: %f", pos_data.x)
        rospy.loginfo("y: %f", pos_data.y)
        rospy.loginfo("z: %f", pos_data.z)
        rospy.loginfo("roll: %f", pos_data.roll)
        rospy.loginfo("pitch: %f", pos_data.pitch)
        rospy.loginfo("yaw: %f", pos_data.yaw)
        rospy.loginfo("gripper: %f", pos_data.gripper)
        rospy.loginfo("mode1: %d", pos_data.mode1)
        rospy.loginfo("mode2: %d", pos_data.mode2)
        x = round(pos_data.x*1000)
        y = round(pos_data.y*1000)
        z = round(pos_data.z*1000)
        rx = round(pos_data.roll*1000)
        ry = round(pos_data.pitch*1000)
        rz = round(pos_data.yaw*1000)
        if(self.GetEnableFlag()):
            self.piper.MotionCtrl_1(0x00, 0x00, 0x00)
            self.piper.MotionCtrl_2(0x01, 0x02, 50)
            self.piper.EndPoseCtrl(x, y, z, 
                                    rx, ry, rz)
            gripper = round(pos_data.gripper*1000*1000)
            if(pos_data.gripper>80000): gripper = 80000
            if(pos_data.gripper<0): gripper = 0
            if(self.girpper_exist):
                self.piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)
            self.piper.MotionCtrl_2(0x01, 0x00, 50)
    
    def joint_callback(self, joint_data):
        """机械臂关节角回调函数

        Args:
            joint_data (): 
        """
        factor = 57324.840764 #1000*180/3.14
        factor1 = 57.32484
        rospy.loginfo("Received Joint States:")
        rospy.loginfo("joint_0: %f", joint_data.position[0])
        rospy.loginfo("joint_1: %f", joint_data.position[1])
        rospy.loginfo("joint_2: %f", joint_data.position[2])
        rospy.loginfo("joint_3: %f", joint_data.position[3])
        rospy.loginfo("joint_4: %f", joint_data.position[4])
        rospy.loginfo("joint_5: %f", joint_data.position[5])
        rospy.loginfo("joint_6: %f", joint_data.position[6])
        joint_0 = round(joint_data.position[0]*factor)
        joint_1 = round(joint_data.position[1]*factor)
        joint_2 = round(joint_data.position[2]*factor)
        joint_3 = round(joint_data.position[3]*factor)
        joint_4 = round(joint_data.position[4]*factor)
        joint_5 = round(joint_data.position[5]*factor)
        joint_6 = round(joint_data.position[6]*1000*1000)
        if(self.rviz_ctrl_flag):
            joint_6 = joint_6 * 2
        if(joint_6>80000): joint_6 = 80000
        if(joint_6<0): joint_6 = 0
        if(self.GetEnableFlag()):
            # 设定电机速度
            if(joint_data.velocity != []):
                all_zeros = all(v == 0 for v in joint_data.velocity)
            else: all_zeros = True
            if(not all_zeros):
                lens = len(joint_data.velocity)
                if(lens == 7):
                    vel_all = round(joint_data.velocity[6])
                    if (vel_all > 100): vel_all = 100
                    if (vel_all < 0): vel_all = 0
                    rospy.loginfo("vel_all: %f", vel_all)
                    self.piper.MotionCtrl_2(0x01, 0x01, vel_all)
                # elif(lens == 7):
                #     # 遍历速度列表
                #     for i, velocity in enumerate(joint_data.velocity):
                #         if velocity > 0:  # 如果速度是正数
                #             # 设置指定位置的关节速度为这个正数速度
                #             # self.piper.SearchMotorMaxAngleSpdAccLimit(i+1,0x01)
                #             # self.piper.MotorAngleLimitMaxSpdSet(i+1)
                else: self.piper.MotionCtrl_2(0x01, 0x01, 30)
            else: self.piper.MotionCtrl_2(0x01, 0x01, 30)
            # 给定关节角位置
            self.piper.JointCtrl(joint_0, joint_1, joint_2, 
                                    joint_3, joint_4, joint_5)
            # 如果末端夹爪存在，则发送末端夹爪控制
            if(self.girpper_exist):
                if(len(joint_data.effort) == 7):
                    gripper_effort = round(joint_data.effort[6])
                    if (gripper_effort > 3): gripper_effort = 3
                    if (gripper_effort < 0.5): gripper_effort = 0.5
                    rospy.loginfo("gripper_effort: %f", gripper_effort)
                    self.piper.GripperCtrl(abs(joint_6), gripper_effort*1000, 0x01, 0)
                # 默认1N
                else: self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
    
    def enable_callback(self, enable_flag:Bool):
        """机械臂使能回调函数

        Args:
            enable_flag (): 
        """
        rospy.loginfo("Received enable flag:")
        rospy.loginfo("enable_flag: %s", enable_flag.data)
        if(enable_flag.data):
            self.__enable_flag = True
            self.piper.EnableArm(7)
            if(self.girpper_exist):
                self.piper.GripperCtrl(0,1000,0x01, 0)
        else:
            self.__enable_flag = False
            self.piper.DisableArm(7)
            if(self.girpper_exist):
                self.piper.GripperCtrl(0,1000,0x00, 0)
    
    def handle_enable_service(self,req):
        rospy.loginfo(f"Received request: {req.enable_request}")
        enable_flag = False
        loop_flag = False
        # 设置超时时间（秒）
        timeout = 5
        # 记录进入循环前的时间
        start_time = time.time()
        elapsed_time_flag = False
        while not (loop_flag):
            elapsed_time = time.time() - start_time
            print("--------------------")
            enable_list = []
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status)
            if(req.enable_request):
                enable_flag = all(enable_list)
                self.piper.EnableArm(7)
                self.piper.GripperCtrl(0,1000,0x01, 0)
            else:
                enable_flag = any(enable_list)
                self.piper.DisableArm(7)
                self.piper.GripperCtrl(0,1000,0x02, 0)
            print("使能状态:", enable_flag)
            self.__enable_flag = enable_flag
            print("--------------------")
            if(enable_flag == req.enable_request):
                loop_flag = True
                enable_flag = True
            else: 
                loop_flag = False
                enable_flag = False
            # 检查是否超过超时时间
            if elapsed_time > timeout:
                print("超时....")
                elapsed_time_flag = True
                enable_flag = False
                loop_flag = True
                break
            time.sleep(0.5)
        response = enable_flag
        rospy.loginfo(f"Returning response: {response}")
        return EnableResponse(response)

if __name__ == '__main__':
    try:
        piper_signle = C_PiperRosNode()
        piper_signle.Pubilsh()
    except rospy.ROSInterruptException:
        pass
