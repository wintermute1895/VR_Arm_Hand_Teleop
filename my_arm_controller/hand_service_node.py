#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 这个脚本必须在 (hand_venv) 环境下运行！

import rospy
from std_msgs.msg import Float32MultiArray # 我们用一个简单的消息类型来通信

# 导入灵巧手SDK的核心库
# from linker_hand_sdk import HandController # 假设SDK是这样用的

class HandServiceNode:
    def __init__(self):
        rospy.init_node('hand_service_node')
        
        # 1. 初始化灵巧手硬件
        # self.hand = HandController(port='/dev/ttyUSB0') # 假设的SDK用法
        # self.hand.connect()
        rospy.loginfo("Linker Hand SDK initialized (running in venv).")

        # 2. 创建一个ROS订阅者，监听来自主应用的指令
        rospy.Subscriber('/hand/command', Float32MultiArray, self.command_callback)
        
        rospy.loginfo("Hand service node is ready and waiting for commands.")
        rospy.spin()

    def command_callback(self, msg):
        # 3. 当收到ROS消息时，将其转换成SDK的指令
        #    msg.data 是一个包含手指角度的数组
        target_angles = list(msg.data)
        rospy.loginfo("Received hand command: %s", str(target_angles))
        
        # 4. 调用SDK来驱动真实硬件
        # self.hand.set_angles(target_angles) # 假设的SDK用法

if __name__ == '__main__':
    try:
        HandServiceNode()
    except rospy.ROSInterruptException:
        pass