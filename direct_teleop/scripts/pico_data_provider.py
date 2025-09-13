#!/usr/bin/env python3
import rospy
import socket
import json
import threading
import time
from geometry_msgs.msg import PoseStamped, Point
from direct_teleop.msg import PicoHand

class PicoJsonDataProvider:
    def __init__(self, udp_ip="0.0.0.0", udp_port=9999):
        rospy.init_node('pico_ros_bridge')

        self.udp_ip = udp_ip
        self.udp_port = udp_port

        # --- [核心修改 1] 创建左右手两个独立的Publisher ---
        self.left_keypoints_pub = rospy.Publisher('/pico/hand_left_data', PicoHand, queue_size=1)
        self.right_keypoints_pub = rospy.Publisher('/pico/hand_right_data', PicoHand, queue_size=1)

        self.lock = threading.Lock()
        self.buffer = ""
        self.udp_thread = threading.Thread(target=self._udp_receiver_loop)
        self.udp_thread.daemon = True
        self.udp_thread.start()

        rospy.loginfo(f"PICO ROS Bridge initialized. Listening on UDP {self.udp_port}. Publishing left/right hand data.")

    def _udp_receiver_loop(self):
        """后台线程，持续接收UDP数据并存入缓冲区。 (保持原始逻辑)"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(1.0) 
        sock.bind((self.udp_ip, self.udp_port))
        
        while not rospy.is_shutdown():
            try:
                data, _ = sock.recvfrom(8192)
                decoded_data = data.decode('utf-8', errors='ignore')
                with self.lock:
                    self.buffer += decoded_data
            except socket.timeout:
                rospy.logwarn_throttle(5.0, "UDP socket timed out. Is PICO sending data?")
                continue
            except Exception as e:
                rospy.logerr_throttle(5.0, f"UDP receiver thread error: {e}")

    def run_publisher_loop(self):
        """主循环，以固定频率处理缓冲区数据并发布ROS消息。"""
        rospy.loginfo("Starting main publisher loop at 60Hz.")
        rate = rospy.Rate(60)
        
        while not rospy.is_shutdown():
            self.process_buffer()
            rate.sleep()

    def process_buffer(self):
        """
        [核心] 恢复到你之前能工作的、基于换行符分割的简单逻辑。
        这个逻辑假定PICO发送的每个JSON消息都以换行符 `\n` 结尾。
        """
        message_to_process = None
        with self.lock:
            # 找到最后一个换行符
            last_newline_idx = self.buffer.rfind('\n')
            if last_newline_idx != -1:
                # 找到倒数第二个换行符，以提取最后一个完整的消息
                prev_newline_idx = self.buffer.rfind('\n', 0, last_newline_idx)
                message_to_process = self.buffer[prev_newline_idx + 1 : last_newline_idx]
                # 清空缓冲区，只保留可能不完整的最后一部分
                self.buffer = self.buffer[last_newline_idx + 1:]

        if message_to_process:
            self.publish_data(message_to_process)

    def publish_data(self, json_str):
        """
        解析JSON字符串并根据手型发布到对应的话题。
        (保留你原始的、能工作的关键点构建逻辑)
        """
        try:
            data_packet = json.loads(json_str)
            
            # --- [核心修改 2] 根据手型选择正确的Publisher ---
            hand_type_str = data_packet.get('hand') # 获取是 'HandLeft' 还是 'HandRight'
            
            publisher = None
            if hand_type_str == 'HandLeft':
                publisher = self.left_keypoints_pub
            elif hand_type_str == 'HandRight':
                publisher = self.right_keypoints_pub
            else:
                # 如果没有手型信息，或者手型不是我们关心的，则直接返回
                return

            # --- 保留你原始的、能工作的关键点构建逻辑 ---
            joints = data_packet.get('joints')
            if not joints or not isinstance(joints, list) or len(joints) < 26: 
                return

            current_time = rospy.Time.now()
            
            hand_msg = PicoHand()
            hand_msg.header.stamp = current_time
            # 假设你的PicoHand.msg期望26个点，且原始joints列表已经是正确的顺序
            keypoints_list = [Point(p.get('posX',0), p.get('posY',0), p.get('posZ',0)) for p in joints]
            
            # 安全检查，确保关键点数量正确
            if len(keypoints_list) == 26:
                hand_msg.keypoints = keypoints_list
                # 使用正确的publisher发布
                publisher.publish(hand_msg)
                rospy.loginfo_throttle(1.0, f"--> SUCCESS: Published data for '{hand_type_str}'.")
            else:
                rospy.logwarn_throttle(2.0, f"Expected 26 keypoints for '{hand_type_str}', but got {len(keypoints_list)}. Skipping.")

        except (json.JSONDecodeError, KeyError, IndexError) as e:
            rospy.logwarn_throttle(2.0, f"Failed to process packet: {e}. Raw: {json_str[:100]}...")

if __name__ == '__main__':
    try:
        provider = PicoJsonDataProvider()
        provider.run_publisher_loop()
    except rospy.ROSInterruptException:
        pass