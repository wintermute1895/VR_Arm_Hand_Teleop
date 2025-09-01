#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# rosrun teleop_nodes teleop_servo_node.py

"""
teleop_servo_node.py

功能:
- 实现了基于速度的实时伺服遥操作。
- 订阅PICO裸手数据 (`/pico/unified_data`)。
- 计算手腕的位姿变化，并将其转换为末端执行器的速度指令 (TwistStamped)。
- 将计算出的速度指令发布到 '/servo_server/delta_twist_cmds' 话题，供 moveit_servo 节点使用。
"""

import rospy
import numpy as np
import sys
from geometry_msgs.msg import TwistStamped
from teleop_nodes.msg import UnifiedDataPacket
from tf.transformations import quaternion_multiply, quaternion_inverse

class TeleopServoNode:
    def __init__(self):
        rospy.init_node('teleop_servo_node', anonymous=True)

        # --- 1. 发布者：发布速度指令给 moveit_servo ---
        # 话题名称与moveit_servo的默认输入匹配
        self.twist_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=1)
        
        # --- 2. 订阅者：接收PICO数据 ---
        self.pico_data_sub = rospy.Subscriber(
            '/pico/unified_data', 
            UnifiedDataPacket, 
            self.pico_data_callback, 
            queue_size=1, 
            tcp_nodelay=True
        )

        # --- 3. 状态变量，用于计算速度 ---
        self.last_pico_pos = None
        self.last_pico_quat = None
        self.last_time = None

        # --- 4. 控制和校准参数 ---
        # **调试点1**: 速度缩放比例。调大它，手的移动会产生更快的机械臂运动。
        self.linear_scale = 0.7
        # **调试点2**: 角度速度缩放比例。
        self.angular_scale = 0.3
        
        # **调试点3**: 你的“手部零点”校准值。
        self.pico_origin_offset = np.array([0.15, 1.60, -0.20])
        
        self.WRIST_JOINT_ID = 0
        
        rospy.loginfo("Teleop Servo Node: Initialized. Ready to send velocity commands.")
        rospy.loginfo("Ensure 'moveit_servo' is running and listening on '/servo_server/delta_twist_cmds'")

    def pico_data_callback(self, data_msg):
        if not data_msg.right_hand_joints: return

        wrist_data = next((joint for joint in data_msg.right_hand_joints if joint.id == self.WRIST_JOINT_ID), None)
        if wrist_data is None: return

        current_time = rospy.Time.now()
        
        # 1. 原始数据获取
        pico_pos_raw = np.array([wrist_data.pos_x, wrist_data.pos_y, wrist_data.pos_z])
        pico_quat_raw = np.array([wrist_data.rot_x, wrist_data.rot_y, wrist_data.rot_z, wrist_data.rot_w])

        # 2. 坐标变换
        pico_pos_relative = pico_pos_raw - self.pico_origin_offset
        current_pico_pos = np.array([pico_pos_relative[2], -pico_pos_relative[0], pico_pos_relative[1]])
        current_pico_quat = pico_quat_raw 

        # 3. 初始化状态 (仅在第一帧)
        if self.last_pico_pos is None:
            self.last_pico_pos = current_pico_pos
            self.last_pico_quat = current_pico_quat
            self.last_time = current_time
            rospy.loginfo("Servo Teleop: First PICO data received, initializing state.")
            return

        # 4. 计算时间差 (dt)
        dt = (current_time - self.last_time).to_sec()
        if dt < 1e-4: return # 避免时间间隔过小导致速度爆炸
            
        # 5. 计算线速度和角速度
        linear_velocity = (current_pico_pos - self.last_pico_pos) / dt
        quat_diff = quaternion_multiply(current_pico_quat, quaternion_inverse(self.last_pico_quat))
        
        # 6. 创建并发布TwistStamped消息
        twist_msg = TwistStamped()
        twist_msg.header.stamp = current_time
        twist_msg.header.frame_id = "base_link"

        twist_msg.twist.linear.x = linear_velocity[0] * self.linear_scale
        twist_msg.twist.linear.y = linear_velocity[1] * self.linear_scale
        twist_msg.twist.linear.z = linear_velocity[2] * self.linear_scale
        
        # 将四元数差转换为角速度 (小角度近似)
        twist_msg.twist.angular.x = (quat_diff[0] * 2 / dt) * self.angular_scale
        twist_msg.twist.angular.y = (quat_diff[1] * 2 / dt) * self.angular_scale
        twist_msg.twist.angular.z = (quat_diff[2] * 2 / dt) * self.angular_scale
        
        self.twist_pub.publish(twist_msg)

        # 7. 更新状态以备下一帧使用
        self.last_pico_pos = current_pico_pos
        self.last_pico_quat = current_pico_quat
        self.last_time = current_time

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TeleopServoNode()
        node.run()
    except rospy.ROSInterruptException:
        pass