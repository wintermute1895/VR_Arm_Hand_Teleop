#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# rosrun teleop_nodes servo_speed_test.py

"""
servo_speed_test.py

功能:
- 这是一个用于测试 moveit_servo 在 'speed_units' 模式下的模拟数据发布器。
- 它会以固定频率，向 '/servo_server/delta_twist_cmds' 话题发布一个
  在 -1.0 到 1.0 之间的比例速度指令。
"""

import rospy
from geometry_msgs.msg import TwistStamped
import sys

def speed_test_publisher():
    # 初始化节点
    rospy.init_node('servo_speed_test_publisher', anonymous=True)

    # 创建发布者，发布到 moveit_servo 订阅的话题
    pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)

    # 设置发布频率 (50Hz)
    rate = rospy.Rate(50)

    rospy.loginfo("Servo Speed Test Publisher: Starting to publish 'speed_units' commands...")
    rospy.loginfo("The robot arm in RViz should start moving very slowly.")

    # 创建一个恒定的速度指令消息
    twist_cmd = TwistStamped()
    twist_cmd.header.frame_id = "base_link" # 速度是在机器人基座坐标系下定义的

    # --- 使用 -1.0 到 1.0 的比例值 ---
    # 发送一个指令，代表“使用最大线性速度的10% (0.1) 向前移动”
    # 这是一个非常小、非常安全的值，便于观察
    twist_cmd.twist.linear.x = 0.1
    twist_cmd.twist.linear.y = 0.0
    twist_cmd.twist.linear.z = 0.0
    
    # 为了让效果更明显，我们也让它同时绕Z轴（基座）旋转
    # 代表“使用最大角速度的5% (0.05) 旋转”
    twist_cmd.twist.angular.x = 0.0
    twist_cmd.twist.angular.y = 0.0
    twist_cmd.twist.angular.z = 0.05
    # ------------------------------------

    while not rospy.is_shutdown():
        # 更新时间戳并发布消息
        twist_cmd.header.stamp = rospy.Time.now()
        pub.publish(twist_cmd)
        
        # 按照设定的频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        speed_test_publisher()
    except rospy.ROSInterruptException:
        pass