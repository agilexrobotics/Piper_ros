#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class JointStateRelay:
    def __init__(self):
        rospy.init_node('rviz_ctrl_gazebo', anonymous=True)

        # 创建发布者来发布到每个关节的控制器话题
        self.joint1_pub = rospy.Publisher('/piper_description/joint1_position_controller/command', Float64, queue_size=2)
        self.joint2_pub = rospy.Publisher('/piper_description/joint2_position_controller/command', Float64, queue_size=2)
        self.joint3_pub = rospy.Publisher('/piper_description/joint3_position_controller/command', Float64, queue_size=2)
        self.joint4_pub = rospy.Publisher('/piper_description/joint4_position_controller/command', Float64, queue_size=2)
        self.joint5_pub = rospy.Publisher('/piper_description/joint5_position_controller/command', Float64, queue_size=2)
        self.joint6_pub = rospy.Publisher('/piper_description/joint6_position_controller/command', Float64, queue_size=2)
        self.joint7_pub = rospy.Publisher('/piper_description/joint7_position_controller/command', Float64, queue_size=2)
        self.joint8_pub = rospy.Publisher('/piper_description/joint8_position_controller/command', Float64, queue_size=2)

        # 订阅/joint_states话题
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

    def joint_states_callback(self, msg):
        # 假设joint_states中的关节名称和控制器的话题顺序一致
        # 如果顺序不同，需要进行映射
        try:
            # 逐个发布到相应的话题
            self.joint1_pub.publish(msg.position[0])
            self.joint2_pub.publish(msg.position[1])
            self.joint3_pub.publish(msg.position[2])
            self.joint4_pub.publish(msg.position[3])
            self.joint5_pub.publish(msg.position[4])
            self.joint6_pub.publish(msg.position[5])
            self.joint7_pub.publish(msg.position[6])
            self.joint8_pub.publish(msg.position[7])
        except IndexError:
            rospy.logerr("Received joint states do not match expected number of joints.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        relay = JointStateRelay()
        relay.run()
    except rospy.ROSInterruptException:
        pass
