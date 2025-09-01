#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
# 假设PICO手部数据通过一个自定义消息类型发布
# 我们将在下一步创建这个消息
from direct_teleop.msg import PicoHand  # 从我们自己的包导入消息

# 从我们刚刚组织的库中导入核心算法类
import direct_teleop.src.hand_retargeting_lib.L21.config as config
from direct_teleop.src.hand_retargeting_lib.L21.robothandmodel import RobotHandModel
from hand_retargeting_lib.retargeting import HandOptimizer
from direct_teleop.src.hand_retargeting_lib.L21.utils_1 import convert_q_to_robot_command_l21

class HandRetargetingNode:
    def __init__(self):
        rospy.init_node('hand_retargeting_node')

        # --- 1. 初始化重定向算法 ---
        self.config = config
        self.robot_model = RobotHandModel(self.config)
        self.optimizer = HandOptimizer(self.robot_model, self.config)
        
        # 标志位，用于首次校准
        self.is_calibrated = False

        # --- 2. 状态变量 ---
        # 存储最新的灵巧手关节角度
        self.hand_joint_names = self._get_all_hand_joint_names()
        self.current_hand_angles_dict = self._get_zero_angles_dict()

        # --- 3. ROS 订阅与发布 ---
        # 订阅来自PICO的原始手部关键点数据
        rospy.Subscriber('/pico/hand_keypoints', PicoHand, self.pico_hand_callback, queue_size=1)
        
        # 发布灵巧手的硬件指令 (0-255)
        # (这将在对接硬件时使用)
        # self.command_pub = rospy.Publisher('/linkerhand/command', ..., queue_size=10)
        
        rospy.loginfo("Hand Retargeting Node is ready.")

    def _get_all_hand_joint_names(self):
        names = []
        for finger in config.FINGER_NAMES:
            names.extend(config.JOINT_INFO[finger]['names'])
        return names

    def _get_zero_angles_dict(self):
        q_dict = {}
        for finger in config.FINGER_NAMES:
            q_dict[finger] = [0.0] * len(config.JOINT_INFO[finger]['names'])
        return q_dict

    def pico_hand_callback(self, msg):
        # 将ROS消息中的Point[]数组转换为 (N, 3) 的Numpy数组
        # 这是您的算法期望的输入格式
        w_lhs = np.array([[p.x, p.y, p.z] for p in msg.keypoints])

        # 首次接收数据时，进行校准
        if not self.is_calibrated:
            self.robot_model.calibrate(w_lhs)
            self.is_calibrated = True
            rospy.loginfo("Hand model calibrated.")
            return

        # 运行核心优化算法
        final_q_dict, loss = self.optimizer.optimize_q(w_lhs)

        if loss != -1.0:
            self.current_hand_angles_dict = final_q_dict
            
            # (可选) 将角度转换为硬件指令并发布
            # command_list = convert_q_to_robot_command_l21(final_q_dict, self.config)
            # self.command_pub.publish(...)
        else:
            rospy.logwarn_throttle(1.0, f"Hand optimization failed.")

    def get_joint_state_msg(self):
        """创建一个只包含手部关节的JointState消息"""
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        
        positions = []
        names = []
        for finger in config.FINGER_NAMES:
            finger_names = config.JOINT_INFO[finger]['names']
            finger_angles = self.current_hand_angles_dict[finger]
            names.extend(finger_names)
            positions.extend(finger_angles)
            
        js_msg.name = names
        js_msg.position = positions
        return js_msg

# 这个节点本身不spin，它会被其他节点调用
# 如果要独立运行，可以添加一个主循环和发布器