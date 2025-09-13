#!/usr/bin/env python3

import rospy
import subprocess
import signal
import os
import time
from std_msgs.msg import String # 用于接收简单的控制指令

# --- 不需要导入的库 (因为不再在节点内部处理图像和DataFrame) ---
# import message_filters
# from sensor_msgs.msg import Image, JointState
# import cv2
# from cv_bridge import CvBridge, CvBridgeError
# import numpy as np 
# import pandas as pd
# from direct_teleop.msg import PicoHand 

class HandDataCollector:
    def __init__(self, save_root_path):
        rospy.init_node('hand_data_collector_controller', anonymous=True) # 节点名更明确为控制器

        # self.bridge = CvBridge() # 不需要了
        
        # --- [修改] 从参数服务器获取目标手型 ---
        self.target_hand_type = rospy.get_param('~target_hand_type', 'left') # 默认左手
        rospy.loginfo(f"Data Collector configured for '{self.target_hand_type}' hand (using rosbag).")

        # --- 数据存储设置 ---
        self.save_root_path = save_root_path
        self.episode_count = self._get_latest_episode_count()
        # self.frame_count = 0 # 不需要了
        self.is_recording = False
        self.rosbag_process = None # 存储 rosbag 进程的句柄
        # self.collected_data = [] # 不需要了

        # --- ROS 订阅 (只订阅控制指令) ---
        rospy.Subscriber('/recording_control', String, self.control_callback)

        # --- rosbag 录制话题列表 ---
        # 动态构建需要录制的话题列表
        self.topics_to_record = [
            '/image_raw', # 摄像头图像
            f'/pico/hand_{self.target_hand_type}_data', # PICO原始关键点
            f'/hand_retargeting/target_joint_states_{self.target_hand_type}', # 算法节点发布的关节指令
            f'/cb_{self.target_hand_type}_hand_control_cmd', # 硬件控制节点发布的指令
            # 你也可以根据需要添加其他话题，例如 /joint_states (机械臂本体关节状态)
            '/joint_states' # 原始机械臂关节状态，这对于模仿学习也很重要
        ]
        
        rospy.loginfo("="*50)
        rospy.loginfo("Hand Data Collector is running.")
        rospy.loginfo(f"Raw rosbag data will be saved in: {self.save_root_path}")
        rospy.loginfo(f"Configured to record for '{self.target_hand_type}' hand.")
        rospy.loginfo("To start recording an episode, run:")
        rospy.loginfo("rostopic pub /recording_control std_msgs/String \"data: 'start'\" -1")
        rospy.loginfo("To stop recording, run:")
        rospy.loginfo("rostopic pub /recording_control std_msgs/String \"data: 'stop'\" -1")
        rospy.loginfo("="*50)

    def _get_latest_episode_count(self):
        """检查已有文件夹，确定新的episode编号"""
        if not os.path.exists(self.save_root_path):
            os.makedirs(self.save_root_path)
        existing_episodes = [d for d in os.listdir(self.save_root_path) if d.startswith('episode_')]
        if not existing_episodes:
            return 0
        # 从文件名中提取数字部分
        return max([int(d.split('_')[1]) for d in existing_episodes if len(d.split('_')) > 1])


    def control_callback(self, msg):
        """接收开始/停止指令"""
        if msg.data == 'start' and not self.is_recording:
            self.start_new_episode()
        elif msg.data == 'stop' and self.is_recording:
            self.stop_and_save_episode()

    def start_new_episode(self):
        """开始一个新的数据采集片段，并启动rosbag录制"""
        self.is_recording = True
        self.episode_count += 1
        # self.frame_count = 0 # 不需要了
        # self.collected_data = [] # 不需要了
        
        # 定义rosbag文件名
        bag_name = os.path.join(self.save_root_path, f"episode_{self.episode_count:04d}_{self.target_hand_type}_{time.strftime('%Y%m%d_%H%M%S')}.bag")
        record_command = ['rosbag', 'record', '-O', bag_name, '--duration=0'] + self.topics_to_record # --duration=0 表示无限录制直到Ctrl+C或接收到信号
        
        try:
            rospy.loginfo(f"Attempting to start rosbag recording for Episode {self.episode_count} to {bag_name}.")
            self.rosbag_process = subprocess.Popen(record_command, preexec_fn=os.setsid) # preexec_fn 确保进程组正确
            rospy.loginfo(f"--- Started rosbag recording for Episode {self.episode_count} to {bag_name}. PID: {self.rosbag_process.pid} ---")
        except Exception as e:
            rospy.logerr(f"Failed to start rosbag: {e}")
            self.rosbag_process = None
            self.is_recording = False # 启动失败则不进入录制状态

    def stop_and_save_episode(self):
        """停止并保存当前片段的数据 (通过停止rosbag)"""
        if not self.is_recording:
            rospy.logwarn("Not currently recording. Ignoring stop command.")
            return

        rospy.loginfo(f"--- Stopping recording Episode {self.episode_count} for {self.target_hand_type} hand ---")
        if self.rosbag_process:
            # os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGINT) # 向进程组发送SIGINT
            self.rosbag_process.send_signal(signal.SIGINT) # Ctrl+C
            self.rosbag_process.wait(timeout=5) # 等待rosbag进程终止，超时5秒
            if self.rosbag_process.poll() is None: # 如果进程仍在运行，强制终止
                rospy.logwarn("Rosbag process did not terminate gracefully, killing it.")
                self.rosbag_process.kill()
            rospy.loginfo("Rosbag recording stopped.")
            self.rosbag_process = None
        else:
            rospy.logwarn("No active rosbag process found.")
        
        self.is_recording = False # 标记为停止录制
        rospy.loginfo(f"--- Episode {self.episode_count} raw data saved to rosbag. ---")

    # [移除] synchronized_callback，因为它不再用于实时保存
    # def synchronized_callback(self, image_msg, pico_keypoints_msg, hand_cmd_msg):
    #     pass # 这个函数现在是空的，或者直接移除

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    # 定义你的数据集要保存的根目录
    # rosbag 会直接保存在这个目录下
    dataset_path = os.path.expanduser("~/robot_hand_raw_bags") 
    
    try:
        collector = HandDataCollector(save_root_path=dataset_path)
        collector.run()
    except rospy.ROSInterruptException:
        pass