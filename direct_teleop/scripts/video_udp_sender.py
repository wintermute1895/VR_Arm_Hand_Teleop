#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import socket
import numpy as np
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  

# --- 配置 ---
PICO_IP = "192.168.1.105"  # !! 重要：这里需要填入PICO设备的IP地址 !!
PORT = 9998                  # 使用一个新端口，避免和手部数据(9999)冲突
VIDEO_DEVICE = 0             # 摄像头的设备号, 通常是0
JPEG_QUALITY = 40            # 图像压缩质量 (0-100)，值越低延迟越小，但画质差
TARGET_FPS = 30              # 目标发送和发布帧率

def main():
    rospy.init_node('video_udp_sender', anonymous=True)
    image_pub = rospy.Publisher('/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(TARGET_FPS) # 定义ROS Rate

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (PICO_IP, PORT)
    print(f"Starting UDP video stream to {PICO_IP}:{PORT}")
    print(f"Publishing images to ROS topic /image_raw")

    # --- 健壮的摄像头初始化逻辑 ---
    cap = cv2.VideoCapture(VIDEO_DEVICE)
    max_retries = 5
    retries = 0
    while not cap.isOpened() and retries < max_retries:
        print(f"Error: Cannot open video device {VIDEO_DEVICE}. Retrying in 1 second...")
        time.sleep(1.0)
        cap = cv2.VideoCapture(VIDEO_DEVICE)
        retries += 1
    
    if not cap.isOpened():
        print(f"Critical Error: Failed to open video device {VIDEO_DEVICE} after {max_retries} retries. Exiting.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    print(f"Successfully opened video device {VIDEO_DEVICE} at {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}.")

    try:
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                print("Error: Can't receive frame from camera. Attempting to re-read...")
                time.sleep(0.1) # 稍等片刻再重试
                ret, frame = cap.read() # 再次尝试读取
                if not ret:
                    print("Failed to receive frame after retry. Skipping this cycle.")
                    rate.sleep()
                    continue # 跳过当前循环，尝试下一帧

            # 将图像帧编码为JPEG格式
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
            result, encoded_image = cv2.imencode('.jpg', frame, encode_param)
            
            # 如果编码成功，则通过UDP发送
            if result:
                try:
                    sock.sendto(encoded_image, server_address)
                except Exception as e:
                    print(f"Network error sending UDP: {e}")
            else:
                print("Image encoding failed, skipping UDP send for this frame.")

            # 发布ROS话题
            try:
                ros_image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image_msg.header.stamp = rospy.Time.now()
                image_pub.publish(ros_image_msg)
            except Exception as e:
                print(f"Error converting or publishing image to ROS: {e}")
            
            rate.sleep() # 控制发送和发布频率

    except KeyboardInterrupt:
        print("Stream stopped by user.")
    finally:
        cap.release()
        sock.close()
        print("Camera and socket released.")


if __name__ == '__main__':
    main()