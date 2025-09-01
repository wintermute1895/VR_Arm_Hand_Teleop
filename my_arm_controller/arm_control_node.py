#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 这个脚本在标准的ROS环境下运行

import rospy
from std_msgs.msg import Float32MultiArray
# ... import moveit_commander etc. ...

class ArmControlNode:
    def __init__(self):
        rospy.init_node('arm_control_node')
        
        # 1. 创建一个ROS发布者，用来给灵巧手服务发指令
        self.hand_pub = rospy.Publisher('/hand/command', Float32MultiArray, queue_size=10)
        
        # ... 初始化MoveIt! commander等 ...
        rospy.loginfo("Arm control node is ready.")

    def some_action(self):
        # 2. 在你的主逻辑里，当你需要控制灵巧手时：
        
        # a. 假设你想让手做一个“抓取”的姿势
        grasp_angles = [1.0, 1.0, 1.0, 1.0, 0.5] # 假设的角度值
        
        # b. 创建ROS消息
        msg = Float32MultiArray()
        msg.data = grasp_angles
        
        # c. 发布消息！
        self.hand_pub.publish(msg)
        rospy.loginfo("Published hand command: %s", str(grasp_angles))
        
        # ... 在这里继续执行控制机械臂的逻辑 ...

if __name__ == '__main__':
    # ... 主程序 ...