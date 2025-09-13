#!/usr/bin/env python3
import rospy
import numpy as np
import traceback
from sensor_msgs.msg import JointState
import message_filters # 用于同步左右手消息

# 导入L10的专属配置
from hand_retargeting_lib.L10.utils_l10 import convert_q_to_l10_command
import hand_retargeting_lib.L10.config_l10 as config # 路径修正

class LinkerHandL10Controller:
    def __init__(self):
        rospy.init_node('linkerhand_l10_controller')
        self.config = config
        self.hardware_msg_type = JointState

        # --- [修改] 从参数服务器获取目标手型 ---
        # 可以是 'left', 'right', 'both'
        self.target_hand_type = rospy.get_param('~target_hand_type', 'right') 
        rospy.loginfo(f"LinkerHand L10 Controller operating in '{self.target_hand_type}' hand mode.")

        self.left_target_sub = None
        self.right_target_sub = None
        self.left_command_pub = None
        self.right_command_pub = None
        self.last_left_cmd = None # 用于存储上次发送的左手指令
        self.last_right_cmd = None # 用于存储上次发送的右手指令

        # --- [修改] 根据 target_hand_type 动态创建订阅者和发布者 ---
        if self.target_hand_type == 'left' or self.target_hand_type == 'both':
            # 订阅左手算法指令
            self.left_target_sub = message_filters.Subscriber(
                '/hand_retargeting/target_joint_states_left', 
                JointState, 
                queue_size=1
            )
            # 发布左手硬件指令
            self.left_command_pub = rospy.Publisher(
                '/cb_left_hand_control_cmd', # 假设硬件话题是固定的cb_left_hand_control_cmd
                self.hardware_msg_type, 
                queue_size=1
            )
            rospy.loginfo("Subscribing to /hand_retargeting/target_joint_states_left and publishing to /cb_left_hand_control_cmd.")

        if self.target_hand_type == 'right' or self.target_hand_type == 'both':
            # 订阅右手算法指令
            self.right_target_sub = message_filters.Subscriber(
                '/hand_retargeting/target_joint_states_right', 
                JointState, 
                queue_size=1
            )
            # 发布右手硬件指令
            self.right_command_pub = rospy.Publisher(
                '/cb_right_hand_control_cmd', # 假设硬件话题是固定的cb_right_hand_control_cmd
                self.hardware_msg_type, 
                queue_size=1
            )
            rospy.loginfo("Subscribing to /hand_retargeting/target_joint_states_right and publishing to /cb_right_hand_control_cmd.")

        # 如果同时控制左右手，需要用 message_filters 同步两个订阅者
        if self.target_hand_type == 'both':
            self.ts = message_filters.ApproximateTimeSynchronizer(
                [self.left_target_sub, self.right_target_sub], 
                queue_size=10, 
                slop=0.05
            )
            self.ts.registerCallback(self.both_hands_target_callback)
        elif self.target_hand_type == 'left':
            self.left_target_sub.registerCallback(lambda msg: self.single_hand_target_callback(msg, 'left'))
        elif self.target_hand_type == 'right':
            self.right_target_sub.registerCallback(lambda msg: self.single_hand_target_callback(msg, 'right'))


        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("LinkerHand L10 Controller is running and waiting for target joint states...")

    def shutdown_hook(self):
        rospy.loginfo("Shutdown signal received. Resetting hand to open position...")
        open_pose_cmd = JointState()
        open_pose_cmd.header.stamp = rospy.Time.now()
        open_pose_cmd.position = [255, 128, 255, 255, 255, 255, 128, 128, 128, 128]
        
        # 根据启用的手型发送复位指令
        if self.left_command_pub:
            self.left_command_pub.publish(open_pose_cmd)
        if self.right_command_pub:
            self.right_command_pub.publish(open_pose_cmd)

        rospy.sleep(0.1) 
        rospy.loginfo("Hand reset command sent. Node is shutting down.")
        
    def _process_joint_state_msg(self, joint_state_msg, hand_type):
        """通用函数，用于处理JointState消息并转换为硬件指令列表"""
        q_dict_full = {}
        current_joint_info = self.config.get_joint_info(hand_type) # 动态获取关节信息
        
        for finger in self.config.ALL_FINGER_NAMES:
            q_dict_full[finger] = [0.0] * len(current_joint_info[finger]['names'])
        
        for i, name in enumerate(joint_state_msg.name):
            for finger, info in current_joint_info.items(): # 动态使用当前手型下的关节信息
                if name in info['names']:
                    joint_idx = info['names'].index(name)
                    q_dict_full[finger][joint_idx] = joint_state_msg.position[i]
                    break
        
        # 调用转换函数，需要传入当前手型
        position_cmd_list = convert_q_to_l10_command(q_dict_full, self.config, hand_type=hand_type)
        return position_cmd_list

    def single_hand_target_callback(self, joint_state_msg, hand_type):
        rospy.loginfo_once(f"--- First complete target joint state received for {hand_type} hand! ---")
        try:
            position_cmd_list = self._process_joint_state_msg(joint_state_msg, hand_type)
            
            hardware_cmd_msg = JointState()
            hardware_cmd_msg.header.stamp = rospy.Time.now()
            hardware_cmd_msg.position = position_cmd_list

            if hand_type == 'left' and self.left_command_pub:
                self.left_command_pub.publish(hardware_cmd_msg)
                self.last_left_cmd = hardware_cmd_msg.position # 存储指令
            elif hand_type == 'right' and self.right_command_pub:
                self.right_command_pub.publish(hardware_cmd_msg)
                self.last_right_cmd = hardware_cmd_msg.position # 存储指令
            
            rospy.loginfo_throttle(0.2, f"Successfully published command for {hand_type} hand: {position_cmd_list}")
                
        except Exception as e:
            rospy.logerr(f"CRITICAL ERROR in {hand_type} target_callback: {e}")
            rospy.logerr(traceback.format_exc())

    def both_hands_target_callback(self, left_joint_state_msg, right_joint_state_msg):
        rospy.loginfo_once("--- First synchronized target joint states received for both hands! ---")
        try:
            # 处理左手
            left_position_cmd_list = self._process_joint_state_msg(left_joint_state_msg, 'left')
            left_hardware_cmd_msg = JointState()
            left_hardware_cmd_msg.header.stamp = rospy.Time.now()
            left_hardware_cmd_msg.position = left_position_cmd_list
            if self.left_command_pub:
                self.left_command_pub.publish(left_hardware_cmd_msg)
                self.last_left_cmd = left_hardware_cmd_msg.position

            # 处理右手
            right_position_cmd_list = self._process_joint_state_msg(right_joint_state_msg, 'right')
            right_hardware_cmd_msg = JointState()
            right_hardware_cmd_msg.header.stamp = rospy.Time.now()
            right_hardware_cmd_msg.position = right_position_cmd_list
            if self.right_command_pub:
                self.right_command_pub.publish(right_hardware_cmd_msg)
                self.last_right_cmd = right_hardware_cmd_msg.position

            rospy.loginfo_throttle(0.2, f"Published commands for both hands. Left: {left_position_cmd_list}, Right: {right_position_cmd_list}")

        except Exception as e:
            rospy.logerr(f"CRITICAL ERROR in both_hands_target_callback: {e}")
            rospy.logerr(traceback.format_exc())

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = LinkerHandL10Controller()
        controller.run()
    except rospy.ROSInterruptException:
        pass