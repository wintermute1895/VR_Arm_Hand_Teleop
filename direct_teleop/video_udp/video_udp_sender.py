import cv2
import socket
import numpy as np
import time

# --- 配置 ---
PICO_IP = "192.168.110.153"  # !! 重要：这里需要填入PICO设备的IP地址 !!
PORT = 9998                  # 使用一个新端口，避免和手部数据(9999)冲突
VIDEO_DEVICE = 0             # 摄像头的设备号, 通常是0
JPEG_QUALITY = 50         # 图像压缩质量 (0-100)，值越低延迟越小

def main():
    # 初始化UDP Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (PICO_IP, PORT)
    print(f"Starting UDP video stream to {PICO_IP}:{PORT}")

    # 初始化摄像头
    cap = cv2.VideoCapture(VIDEO_DEVICE)
    if not cap.isOpened():
        print(f"Error: Cannot open video device {VIDEO_DEVICE}")
        return
        
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Can't receive frame. Exiting ...")
                break

            # 将图像帧编码为JPEG格式
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
            result, encoded_image = cv2.imencode('.jpg', frame, encode_param)
            
            if not result:
                continue

            # 发送数据
            try:
                sock.sendto(encoded_image, server_address)
            except Exception as e:
                print(f"Network error: {e}")
            
            # 控制发送频率，避免网络拥堵 (例如 30 FPS)
            time.sleep(1/30.0)

    except KeyboardInterrupt:
        print("Stream stopped by user.")
    finally:
        cap.release()
        sock.close()
        print("Camera and socket released.")

if __name__ == '__main__':
    main()