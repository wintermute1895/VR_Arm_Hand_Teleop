#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import traceback
import sys, select, tty, termios
import time
from queue import Queue, Empty

from sensor_msgs.msg import JointState
from direct_teleop.msg import PicoHand, LossInfo
from geometry_msgs.msg import PoseStamped

# 导入所有需要的模块
from hand_retargeting_lib.L10 import config_l10 as config
from hand_retargeting_lib.L10.robothandmodel_l10 import RobotHandModel_L10 as RobotHandModel
from hand_retargeting_lib.retargeting import HandOptimizer
from hand_retargeting_lib.L10.utils_l10 import angles_dict_to_vector

class HandRetargetingAlgorithmNode:
    def __init__(self):
        rospy.init_node('hand_retargeting_algorithm_node')
        rospy.loginfo("--- Initializing HandRetargetingAlgorithmNode (Final Stable Version) ---")
        try:
            self.config = config
            self.calibration_state = 0
            self.robot_model = None
            self.optimizer = None
            self.frame_count = 0
            self.last_successful_q_vec = None
            
            # 使用一个队列专门接收真正需要的数据
            self.pico_keypoints_queue = Queue(maxsize=10) # 适当的缓冲大小

            self.hand_joint_pub = rospy.Publisher('/hand_retargeting/target_joint_states', JointState, queue_size=1)
            self.loss_pub = rospy.Publisher('/debug/loss_info', LossInfo, queue_size=1)
            
            # 只订阅手部重定向算法必要的话题
            rospy.Subscriber('/pico/hand/keypoints', PicoHand, self.pico_keypoints_callback, queue_size=1, buff_size=2**24)
            
            rospy.loginfo(">>>> Node initialization COMPLETE. <<<<")
        except Exception as e:
            rospy.logerr(f"CRITICAL ERROR during __init__: {e}")
            rospy.logerr(traceback.format_exc())

    def pico_keypoints_callback(self, msg):
        """
        ROS回调函数，在独立的线程中运行。
        使用put_nowait来避免任何可能的阻塞。
        """
        try:
            self.pico_keypoints_queue.put_nowait(msg)
        except Exception:
            # 如果队列已满，则简单地丢弃这个最旧的数据帧，这符合低延迟的目标
            pass

    def _pico_array_to_hand_dict(self, pico_array):
        if not isinstance(pico_array, np.ndarray) or pico_array.shape[0] < 26: return None
        hand_dict = {'wrist': np.expand_dims(pico_array[self.config.PICO_WRIST_ID], axis=0)}
        for finger, id_list in self.config.PICO_FINGER_KEYPOINT_IDS.items():
            hand_dict[finger] = pico_array[id_list]
        return hand_dict

    def run(self):
        rate = rospy.Rate(50) # 目标循环频率50Hz
        rospy.loginfo("\n" + "="*60 + "\nPress 'c' IN THIS TERMINAL to calibrate.\n" + "="*60)
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not rospy.is_shutdown():
                t_loop_start = time.time()

                # --- 非阻塞式键盘输入检测 ---
                if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                    if sys.stdin.read(1).lower() == 'c' and self.calibration_state == 0:
                        self.calibration_state = 1
                        rospy.loginfo(">>>> 'c' key pressed! Attempting to calibrate... <<<<")
                
                # --- 从队列中获取最新数据 ---
                latest_keypoints_msg = None
                try:
                    # 清空队列，只保留最新的一个消息
                    while not self.pico_keypoints_queue.empty():
                        latest_keypoints_msg = self.pico_keypoints_queue.get_nowait()
                except Empty:
                    pass

                # 如果没有新数据，则跳过本次循环，避免空转
                if latest_keypoints_msg is None:
                    rate.sleep()
                    continue
                
                current_pico_array = np.array([[p.x, p.y, p.z] for p in latest_keypoints_msg.keypoints])
                current_pico_timestamp = latest_keypoints_msg.header.stamp

                # --- 状态机：处理标定 ---
                if self.calibration_state == 1:
                    try:
                        w0_dict = self._pico_array_to_hand_dict(current_pico_array)
                        if w0_dict is None: raise ValueError("Initial PICO data for calibration is invalid.")
                        
                        q0_dict = {f: [0.0]*len(self.config.JOINT_INFO[f]['names']) for f in self.config.ALL_FINGER_NAMES}
                        
                        self.robot_model = RobotHandModel(self.config, q0_dict, w0_dict)
                        self.robot_model.calibrate(w0_dict)
                        self.optimizer = HandOptimizer(self.robot_model, self.config)
                        
                        self.last_successful_q_vec = angles_dict_to_vector(q0_dict, self.config)

                        self.calibration_state = 2
                        rospy.loginfo(">>>> CALIBRATION SUCCESSFUL! Starting retargeting... <<<<")
                    except Exception as e_calib:
                        rospy.logerr(f"Calibration FAILED: {e_calib}. Please move hand into view and press 'c' again.")
                        self.calibration_state = 0
                    rate.sleep()
                    continue

                # --- 状态机：主循环运行 ---
                if self.calibration_state == 2:
                    try:
                        self.frame_count += 1
                        w_dict_current = self._pico_array_to_hand_dict(current_pico_array)
                        if w_dict_current is None:
                            rate.sleep()
                            continue
                        
                        t_before_opt = time.time()
                        final_q_dict, loss_info = self.optimizer.optimize_q(w_dict_current, self.last_successful_q_vec, self.frame_count)
                        t_after_opt = time.time()

                        self.robot_model.update_robot_hand_state(final_q_dict)

                        if loss_info["status"] == "success":
                            self.last_successful_q_vec = angles_dict_to_vector(final_q_dict, self.config)                           
                            self.publish_joint_states(final_q_dict)
                            self.publish_loss_info(loss_info)
                        else:
                            rospy.logwarn_throttle(1.0, f"Optimization FAILED: {loss_info.get('message', 'N/A')}")
                        
                        t_loop_end = time.time()
                        pico_to_now_delay = (rospy.Time.now() - current_pico_timestamp).to_sec()
                        rospy.loginfo_throttle(0.2, 
                            f"--- LATENCY (ms) --- "
                            f"PICO-to-Now: {pico_to_now_delay*1000:5.1f} | "
                            f"Total Loop: {(t_loop_end - t_loop_start)*1000:5.1f} | "
                            f"Optimization: {(t_after_opt - t_before_opt)*1000:.1f}"
                        )
                    except Exception as e_main:
                        rospy.logerr(f"CRITICAL ERROR in main loop: {e_main}")
                        rospy.logerr(traceback.format_exc())
                
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