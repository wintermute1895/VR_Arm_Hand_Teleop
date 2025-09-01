#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import os
import time

def clear_screen():
    """清空终端屏幕，以便刷新显示"""
    os.system('cls' if os.name == 'nt' else 'clear')

def pico_pose_callback(msg):
    """
    回调函数，在收到新消息时被调用，用于刷新并打印位姿。
    """
    pos = msg.pose.position
    quat = msg.pose.orientation
    
    # 清空屏幕
    clear_screen()
    
    # 打印提示信息和数据
    print("="*50)
    print("PICO POSE LISTENER - REAL-TIME DATA")
    print("="*50)
    print(f"Press Ctrl+C to stop the listener when you are ready to copy.\n")
    print(f"Timestamp: {msg.header.stamp.to_sec():.3f}")
    print(f"Frame ID: {msg.header.frame_id}")
    
    print("\n--- Position (for pico_pos list) ---")
    print(f"[{pos.x:.8f}, {pos.y:.8f}, {pos.z:.8f}]")
    
    print("\n--- Orientation Quaternion (for pico_quat list) ---")
    print(f"[{quat.x:.8f}, {quat.y:.8f}, {quat.z:.8f}, {quat.w:.8f}]")
    print("\n" + "="*50)
    
def start_listener():
    """
    初始化节点并订阅话题。
    """
    rospy.init_node('pico_pose_listener', anonymous=True)
    rospy.Subscriber('/pico/hand/pose', PoseStamped, pico_pose_callback)
    
    rospy.loginfo("Listening to /pico/hand/pose...")
    rospy.loginfo("Move your PICO controller to the calibration point.")
    rospy.loginfo("The screen will refresh with the latest data.")
    
    rospy.spin()

if __name__ == '__main__':
    # 在运行此脚本前，请确保 roscore 和您的PICO数据发布节点正在运行
    start_listener()