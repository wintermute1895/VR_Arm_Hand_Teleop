#!/usr/bin/env python3

import rospy
import message_filters
from sensor_msgs.msg import Image, JointState
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import pandas as pd
import os
import time
from std_msgs.msg import String # 用于接收简单的控制指令

class HandDataCollector:
    def __init__(self, save_root_path):
        rospy.init_node('hand_data_collector', anonymous=True)

        # --- 核心组件 ---
        self.bridge = CvBridge()

        # --- 数据存储设置 ---
        self.save_root_path = save_root_path
        self.current_episode_path = ""
        self.episode_count = self._get_latest_episode_count()
        self.frame_count = 0
        self.is_recording = False
        self.collected_data = []

        # --- ROS 订阅与同步 ---
        # 1. 创建订阅者
        image_sub = message_filters.Subscriber('/image_raw', Image)
        hand_cmd_sub = message_filters.Subscriber('/cb_left_hand_control_cmd', JointState)

        # 2. 使用 ApproximateTimeSynchronizer 进行同步
        # queue_size: 内部缓冲区大小
        # slop: 允许消息时间戳的最大差异（秒），这是关键参数！
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, hand_cmd_sub], 
            queue_size=30, 
            slop=0.05  # 50ms的容忍度，对于30Hz的摄像头和50Hz的控制器来说足够了
        )

        # 3. 注册同步回调函数
        self.ts.registerCallback(self.synchronized_callback)

        # 4. 创建一个控制话题，用于开始/停止录制
        rospy.Subscriber('/recording_control', String, self.control_callback)

        rospy.loginfo("="*50)
        rospy.loginfo("Hand Data Collector is running.")
        rospy.loginfo(f"Data will be saved in: {self.save_root_path}")
        rospy.loginfo("To start recording an episode, run:")
        rospy.loginfo("rostopic pub /recording_control std_msgs/String \"data: 'start'\"")
        rospy.loginfo("To stop and save, run:")
        rospy.loginfo("rostopic pub /recording_control std_msgs/String \"data: 'stop'\"")
        rospy.loginfo("="*50)

    def _get_latest_episode_count(self):
        """检查已有文件夹，确定新的episode编号"""
        if not os.path.exists(self.save_root_path):
            os.makedirs(self.save_root_path)
        existing_episodes = [d for d in os.listdir(self.save_root_path) if d.startswith('episode_')]
        if not existing_episodes:
            return 0
        return max([int(d.split('_')[1]) for d in existing_episodes])

    def control_callback(self, msg):
        """接收开始/停止指令"""
        if msg.data == 'start' and not self.is_recording:
            self.start_new_episode()
        elif msg.data == 'stop' and self.is_recording:
            self.stop_and_save_episode()

    def start_new_episode(self):
        """开始一个新的数据采集片段"""
        self.is_recording = True
        self.episode_count += 1
        self.frame_count = 0
        self.collected_data = []
        
        self.current_episode_path = os.path.join(self.save_root_path, f"episode_{self.episode_count:04d}")
        os.makedirs(os.path.join(self.current_episode_path, "images"), exist_ok=True)
        
        rospy.loginfo(f"--- Started recording Episode {self.episode_count} ---")

    def stop_and_save_episode(self):
        """停止并保存当前片段的数据"""
        self.is_recording = False
        
        if not self.collected_data:
            rospy.logwarn("No data collected in this episode. Nothing to save.")
            return

        # 将列表转换为Pandas DataFrame并保存
        df = pd.DataFrame(self.collected_data)
        csv_path = os.path.join(self.current_episode_path, "hand_actions.csv")
        df.to_csv(csv_path, index=False)
        
        rospy.loginfo(f"--- Stopped recording. Saved {self.frame_count} frames to {self.current_episode_path} ---")

    def synchronized_callback(self, image_msg, hand_cmd_msg):
        """这是核心函数，只有当消息同步时才会被调用"""
        if not self.is_recording:
            return

        try:
            # 1. 处理图像
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            image_filename = f"{self.frame_count:05d}.jpg"
            image_save_path = os.path.join(self.current_episode_path, "images", image_filename)
            cv2.imwrite(image_save_path, cv_image)

            # 2. 提取手部指令
            # 确保你的JointState消息的position字段包含了10个关节角度
            joint_positions = list(hand_cmd_msg.position)
            if len(joint_positions) != 10:
                rospy.logwarn_throttle(5.0, f"Expected 10 joint positions, but received {len(joint_positions)}. Skipping frame.")
                return

            # 3. 将数据添加到临时列表中
            row_data = {'image_filename': image_filename}
            for i, pos in enumerate(joint_positions):
                row_data[f'joint_{i}'] = pos
            
            self.collected_data.append(row_data)
            self.frame_count += 1

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"An error occurred in callback: {e}")
            
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    # 定义你的数据集要保存的根目录
    dataset_path = os.path.expanduser("~/robot_hand_dataset") 
    
    try:
        collector = HandDataCollector(save_root_path=dataset_path)
        collector.run()
    except rospy.ROSInterruptException:
        pass