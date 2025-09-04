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
        rospy.init_node('pico_data_provider')

        self.target_hand = rospy.get_param('~target_hand', 'HandLeft')
        self.udp_ip = udp_ip
        self.udp_port = udp_port

        self.pose_pub = rospy.Publisher('/pico/hand/pose', PoseStamped, queue_size=1)
        self.keypoints_pub = rospy.Publisher('/pico/hand/keypoints', PicoHand, queue_size=1)
        
        self.lock = threading.Lock()
        self.buffer = ""
        self.udp_thread = threading.Thread(target=self._udp_receiver_loop)
        self.udp_thread.daemon = True
        self.udp_thread.start()

        rospy.loginfo(f"PICO Provider initialized for '{self.target_hand}'. Listening on UDP {self.udp_port}.")

    def _udp_receiver_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 增加socket超时，防止在某些情况下永久阻塞
        sock.settimeout(1.0) 
        sock.bind((self.udp_ip, self.udp_port))
        
        while not rospy.is_shutdown():
            try:
                t_before_recv = time.time()
                data, _ = sock.recvfrom(8192)
                t_after_recv = time.time()
                
                decoded_data = data.decode('utf-8', errors='ignore')
                
                t_before_lock = time.time()
                with self.lock:
                    self.buffer += decoded_data
                t_after_lock = time.time()
                
                # 打印后台线程的耗时
                rospy.loginfo_throttle(1.0, f"[THREAD DEBUG (ms)] Recv Wait: {(t_after_recv - t_before_recv)*1000:.2f}, Lock Wait: {(t_after_lock - t_before_lock)*1000:.2f}")

            except socket.timeout:
                rospy.logwarn_throttle(5.0, "UDP socket timed out. Is PICO sending data?")
                continue
            except Exception as e:
                rospy.logerr_throttle(5.0, f"UDP receiver thread error: {e}")

    def run_publisher_loop(self):
        rospy.loginfo("Starting main publisher loop at 100Hz.")
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown():
            t_loop_start = time.time()
            
            self.process_buffer()
            
            t_loop_end = time.time()
            rospy.loginfo_throttle(1.0, f"[MAIN LOOP DEBUG (ms)] Total Loop Time: {(t_loop_end - t_loop_start)*1000:.2f}")
            
            rate.sleep()

    def process_buffer(self):
        message_to_process = None
        
        t_before_lock = time.time()
        with self.lock:
            last_newline_idx = self.buffer.rfind('\n')
            if last_newline_idx != -1:
                # 只取最新的一个完整消息
                # 找到倒数第二个换行符
                prev_newline_idx = self.buffer.rfind('\n', 0, last_newline_idx)
                message_to_process = self.buffer[prev_newline_idx + 1 : last_newline_idx]
                # 清空缓冲区，只保留可能不完整的最后一部分
                self.buffer = self.buffer[last_newline_idx + 1:]
        t_after_lock = time.time()

        if message_to_process:
            t_before_pub = time.time()
            self.publish_data(message_to_process)
            t_after_pub = time.time()
            
            # 打印主线程各部分耗时
            rospy.loginfo_throttle(1.0, f"[PROCESS DEBUG (ms)] Lock Wait: {(t_after_lock - t_before_lock)*1000:.2f}, Publish Data: {(t_after_pub - t_before_pub)*1000:.2f}")

    def publish_data(self, json_str):
        try:
            data_packet = json.loads(json_str)
            if data_packet.get('hand') != self.target_hand: return

            joints = data_packet.get('joints')
            if not joints or not isinstance(joints, list) or len(joints) < 26: return

            current_time = rospy.Time.now()
            
            wrist_data = next((joint for joint in joints if joint.get('id') == 1), None)
            if wrist_data:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = current_time
                pose_msg.header.frame_id = "pico_world"
                pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = wrist_data.get('posX',0), wrist_data.get('posY',0), wrist_data.get('posZ',0)
                pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = wrist_data.get('rotX',0), wrist_data.get('rotY',0), wrist_data.get('rotZ',0), wrist_data.get('rotW',1)
                self.pose_pub.publish(pose_msg)

            hand_msg = PicoHand()
            hand_msg.header.stamp = current_time
            joints.sort(key=lambda j: j.get('id', 99))
            keypoints_list = [Point(p.get('posX',0), p.get('posY',0), p.get('posZ',0)) for p in joints]
            hand_msg.keypoints = keypoints_list
            rospy.loginfo(f"PROV: Publishing message with stamp {hand_msg.header.stamp.to_sec():.4f}")
            self.keypoints_pub.publish(hand_msg)

            rospy.loginfo_throttle(1.0, f"--> SUCCESS: Published data for '{self.target_hand}'.") # 改为无节流打印，强制暴露问题

        except (json.JSONDecodeError, KeyError, IndexError) as e:
            rospy.logwarn_throttle(2.0, f"Failed to process packet: {e}.")

if __name__ == '__main__':
    try:
        provider = PicoJsonDataProvider()
        provider.run_publisher_loop()
    except rospy.ROSInterruptException:
        pass