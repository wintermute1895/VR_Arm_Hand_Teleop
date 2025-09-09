#!/usr/bin/env python3
import rospy
import numpy as np
import traceback
from sensor_msgs.msg import JointState

from src.hand_retargeting_lib.L10.utils_l10 import convert_q_to_l10_command
import direct_teleop.src.hand_retargeting_lib.L10.left.config_l10 as config

class LinkerHandL10Controller:
    def __init__(self):
        rospy.init_node('linkerhand_l10_controller')
        self.config = config
        self.hardware_topic = '/cb_left_hand_control_cmd'
        self.hardware_msg_type = JointState

        self.target_sub = rospy.Subscriber(
            '/hand_retargeting/target_joint_states', 
            JointState, 
            self.target_callback, 
            queue_size=1
        )
        self.command_pub = rospy.Publisher(
            self.hardware_topic, 
            self.hardware_msg_type, 
            queue_size=1
        )
        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("LinkerHand L10 Controller is running and waiting for target joint states...")

    def shutdown_hook(self):
        rospy.loginfo("Shutdown signal received. Resetting hand to open position...")
        open_pose_cmd = JointState()
        open_pose_cmd.header.stamp = rospy.Time.now()
        open_pose_cmd.position = [255, 128, 255, 255, 255, 255, 128, 128, 128, 128]
        self.command_pub.publish(open_pose_cmd)
        rospy.sleep(0.1) 
        rospy.loginfo("Hand reset command sent. Node is shutting down.")
        
    def target_callback(self, joint_state_msg):
        rospy.loginfo_once("--- First complete target joint state received! ---")
        try:
            # 1. 将收到的JointState消息转换为完整的q_dict
            q_dict_full = {}
            for finger in self.config.ALL_FINGER_NAMES:
                q_dict_full[finger] = [0.0] * len(self.config.JOINT_INFO[finger]['names'])
            for i, name in enumerate(joint_state_msg.name):
                for finger, info in self.config.JOINT_INFO.items():
                    if name in info['names']:
                        joint_idx = info['names'].index(name)
                        q_dict_full[finger][joint_idx] = joint_state_msg.position[i]
                        break
            
            # 2. 调用转换函数
            position_cmd_list = convert_q_to_l10_command(q_dict_full, self.config)
            
            # --- !! 核心修正：补全被删除的发布逻辑 !! ---
            
            # 3. 创建并填充硬件指令消息
            hardware_cmd_msg = JointState()
            hardware_cmd_msg.header.stamp = rospy.Time.now()
            hardware_cmd_msg.name = [] 
            hardware_cmd_msg.position = position_cmd_list
            hardware_cmd_msg.velocity = []
            hardware_cmd_msg.effort = []

            # 4. 发布指令
            self.command_pub.publish(hardware_cmd_msg)
            
            # 5. (可选) 加入日志以确认发布
            rospy.loginfo_throttle(0.2, f"Successfully published command: {position_cmd_list}")
                
        except Exception as e:
            rospy.logerr(f"CRITICAL ERROR in target_callback: {e}")
            rospy.logerr(traceback.format_exc())

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = LinkerHandL10Controller()
        controller.run()
    except rospy.ROSInterruptException:
        pass