#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import traceback
import sys
import select
import tty
import termios
import time
from datetime import datetime
from sensor_msgs.msg import JointState
from direct_teleop.msg import PicoHand, LossInfo
import threading

# 导入适配层和算法
from hand_retargeting_lib.L10 import config_l10 as config
from hand_retargeting_lib.L10.robothandmodel_l10 import RobotHandModel_L10 as RobotHandModel
from hand_retargeting_lib.retargeting import HandOptimizer
from hand_retargeting_lib.L10.utils_l10 import angles_dict_to_vector # 导入这个工具

class HandRetargetingAlgorithmNode:
    def __init__(self):
        rospy.init_node('hand_retargeting_algorithm_node')
        
        rospy.loginfo("--- Initializing HandRetargetingAlgorithmNode (Active Pull & Key-Triggered) ---")
        try:
            self.config = config
            self.calibration_state = 0
            self.robot_model = None
            self.lock = threading.Lock()
            self.frame_count = 0
            self.latest_pico_array = None
            self.last_successful_q_vec = None
            self.latest_pico_timestamp = None # 新增：用于延迟计算

            self.hand_joint_pub = rospy.Publisher('/hand_retargeting/target_joint_states', JointState, queue_size=1)
            self.loss_pub = rospy.Publisher('/debug/loss_info', LossInfo, queue_size=1)
            rospy.Subscriber('/pico/hand/keypoints', PicoHand, self.pico_data_buffer_callback, queue_size=1)
            rospy.loginfo(">>>> Node initialization COMPLETE. <<<<")

        except Exception as e:
            rospy.logerr(f"CRITICAL ERROR during __init__: {e}")
            rospy.logerr(traceback.format_exc())

    def _pico_array_to_hand_dict(self, pico_array):
        if not isinstance(pico_array, np.ndarray) or pico_array.shape[0] < 26: return None
        hand_dict = {'wrist': np.expand_dims(pico_array[self.config.PICO_WRIST_ID], axis=0)}
        for finger, id_list in self.config.PICO_FINGER_KEYPOINT_IDS.items():
            finger_name = 'pinky' if finger == 'little' else finger
            hand_dict[finger_name] = pico_array[id_list]
        return hand_dict

    def pico_data_buffer_callback(self, msg):
        with self.lock:
            self.latest_pico_array = np.array([[p.x, p.y, p.z] for p in msg.keypoints])
            self.latest_pico_timestamp = msg.header.stamp # 记录PICO数据的时间戳

    def run(self):
        rate = rospy.Rate(50)
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("      PICO HAND RETARGETING CALIBRATION REQUIRED")
        rospy.loginfo("Please place your hand in a flat, open, palm-facing-forward pose.")
        rospy.loginfo("      Then, press 'c' IN THIS TERMINAL to calibrate.")
        rospy.loginfo("="*60 + "\n")
        
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not rospy.is_shutdown():
                t_loop_start = time.time()

                if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                    if sys.stdin.read(1).lower() == 'c' and self.calibration_state == 0:
                        self.calibration_state = 1
                        rospy.loginfo(">>>> 'c' key pressed! Attempting to calibrate... <<<<")
                with self.lock:
                    if self.latest_pico_array is None:
                        rate.sleep(); continue
                    current_pico_array = self.latest_pico_array

                if self.calibration_state == 1:
                    try:
                        w0_dict = self._pico_array_to_hand_dict(current_pico_array)
                        q0_dict = {f: [0.0]*len(self.config.JOINT_INFO[f]['names']) for f in self.config.ALL_FINGER_NAMES}
                        self.robot_model = RobotHandModel(self.config, q0_dict, w0_dict)
                        self.last_successful_q_vec = angles_dict_to_vector(q0_dict, self.config)
                        self.robot_model.calibrate(w0_dict)
                        self.calibration_state = 2
                        rospy.loginfo(">>>> CALIBRATION SUCCESSFUL! <<<<")
                    except Exception as e_calib:
                        rospy.logerr(f"Calibration FAILED: {e_calib}. Please retry.")
                        self.calibration_state = 0
                    rate.sleep(); continue

                if self.calibration_state == 2:
                    try:
                        t_before_process = time.time()
                        w_dict_current = self._pico_array_to_hand_dict(current_pico_array)
                        if w_dict_current is None:
                            rate.sleep(); continue
                        
                        optimizer_this_frame = HandOptimizer(self.robot_model, self.config)

                        t_before_opt = time.time()
                        final_q_dict, loss_info = optimizer_this_frame.optimize_q(w_dict_current, self.last_successful_q_vec)
                        t_after_opt = time.time()

                        self.robot_model.update_robot_hand_state(final_q_dict) # 传入None，因为不再需要内部状态

                        if loss_info["status"] == "success":
                            self.last_successful_q_vec = angles_dict_to_vector(final_q_dict, self.config)                            
                            self.publish_joint_states(final_q_dict)
                            self.publish_loss_info(loss_info)
                        else:
                            rospy.logwarn_throttle(1.0, f"Optimization FAILED: {loss_info.get('message', 'Unknown reason')}")
                        
                        t_loop_end = time.time()
                        # --- !! 新增：完整的延迟测量与打印 !! ---
                        pico_to_now_delay = (rospy.Time.now() - self.latest_pico_timestamp).to_sec()
                        rospy.loginfo_throttle(1.0, 
                            f"--- LATENCY (ms) --- "
                            f"PICO-to-Now: {pico_to_now_delay*1000:5.1f} | "
                            f"Total Loop: {(t_loop_end - t_loop_start)*1000:5.1f} | "
                            f"Data Prep: {(t_before_opt - t_before_process)*1000:5.1f} | "
                            f"Optimization: {(t_after_opt - t_before_opt)*1000:5.1f}"
                        )
                    except Exception as e_main:
                        rospy.logerr(f"CRITICAL ERROR in main loop: {e_main}")
                        rospy.logerr(traceback.format_exc())
                
                self.frame_count += 1
                rate.sleep()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def publish_joint_states(self, q_dict):
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        positions, names = [], []
        for finger in self.config.FINGER_NAMES:
            if finger in q_dict:
                names.extend(self.config.JOINT_INFO[finger]['names'])
                positions.extend(q_dict[finger])
        js_msg.name = names
        js_msg.position = positions
        self.hand_joint_pub.publish(js_msg)

    def publish_loss_info(self, loss_info_dict):
        loss_msg = LossInfo()
        loss_msg.header.stamp = rospy.Time.now()
        loss_msg.total_loss = loss_info_dict.get('total', 0.0)
        loss_msg.loss_align = loss_info_dict.get('align', 0.0)
        loss_msg.loss_couple = loss_info_dict.get('couple', 0.0) 
        loss_msg.loss_smooth = loss_info_dict.get('smooth', 0.0)
        self.loss_pub.publish(loss_msg)

if __name__ == '__main__':
    try:
        node = HandRetargetingAlgorithmNode()
        node.run()
    except rospy.ROSInterruptException:
        pass