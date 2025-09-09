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
JPEG_QUALITY = 40         # 图像压缩质量 (0-100)，值越低延迟越小

def main():
    # 初始化UDP Socket
    rospy.init_node('video_udp_sender', anonymous=True)
    image_pub = rospy.Publisher('/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (PICO_IP, PORT)
    print(f"Starting UDP video stream to {PICO_IP}:{PORT}")
    print(f"Publishing images to ROS topic /image_raw")

    # 初始化摄像头
    cap = cv2.VideoCapture(VIDEO_DEVICE)
    if not cap.isOpened():
        print(f"Error: Cannot open video device {VIDEO_DEVICE}")
        return
        
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    try:
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                print("Error: Can't receive frame. Exiting ...")
                break

            # 将图像帧编码为JPEG格式
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
            result, encoded_image = cv2.imencode('.jpg', frame, encode_param)
            
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        result, encoded_image = cv2.imencode('.jpg', frame, encode_param)

        # 如果编码成功，则通过UDP发送
        if result:
            try:
                sock.sendto(encoded_image, server_address)
            except Exception as e:
                print(f"Network error: {e}")
        else:
            print("Image encoding failed, skipping UDP send for this frame.")

        # 发布ROS话题（无论编码是否成功，只要帧读取成功就发布）
        try:
            ros_image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image_msg.header.stamp = rospy.Time.now()
            image_pub.publish(ros_image_msg)
        except Exception as e:
            print(f"Error converting or publishing image: {e}")
            
            # 控制发送频率，避免网络拥堵 (例如 30 FPS)
            rate = rospy
            rate.sleep()

    except KeyboardInterrupt:
        print("Stream stopped by user.")
    finally:
        cap.release()
        sock.close()
        print("Camera and socket released.")

if __name__ == '__main__':
    main()