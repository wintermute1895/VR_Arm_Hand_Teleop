#!/usr/bin/env python3
import rospy
import socket
import json
import threading
from geometry_msgs.msg import PoseStamped, Point
from direct_teleop.msg import PicoHand

class PicoJsonDataProvider:
    def __init__(self, udp_ip="0.0.0.0", udp_port=9999):
        rospy.init_node('pico_data_provider')

        # --- 1. 参数配置 ---
        self.target_hand = rospy.get_param('~target_hand', 'HandLeft')
        self.udp_ip = udp_ip
        self.udp_port = udp_port

        # --- 2. ROS 发布器 ---
        self.pose_pub = rospy.Publisher('/pico/hand/pose', PoseStamped, queue_size=10)
        self.keypoints_pub = rospy.Publisher('/pico/hand/keypoints', PicoHand, queue_size=10)
        
        # --- 3. 状态变量和线程 ---
        self.latest_complete_message = None
        self.lock = threading.Lock()
        self.buffer = ""
        self.udp_thread = threading.Thread(target=self._udp_receiver_loop)
        self.udp_thread.daemon = True
        self.udp_thread.start()

        rospy.loginfo(f"PICO Provider initialized for '{self.target_hand}'. Listening on UDP {self.udp_port}.")

    def _udp_receiver_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.udp_ip, self.udp_port))
        while not rospy.is_shutdown():
            try:
                data, addr = sock.recvfrom(8192)
                with self.lock:
                    self.buffer += data.decode('utf-8', errors='ignore')
            except Exception as e:
                rospy.logerr_throttle(5.0, f"UDP receiver thread error: {e}")

    def run_publisher_loop(self):
        rospy.loginfo("Waiting 1 second for ROS connections to establish...")
        rospy.sleep(1.0)
        rospy.loginfo("Connections likely established. Starting main publisher loop at 50Hz.")

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.process_buffer()
            rate.sleep()

    def process_buffer(self):
        message_to_process = None
        with self.lock:
            if '\n' in self.buffer:
                lines = self.buffer.strip().split('\n')
                message_to_process = lines[-1]
                self.buffer = ""
        
        if message_to_process:
            self.publish_data(message_to_process)

    def publish_data(self, json_str):
        try:
            data_packet = json.loads(json_str)

            if data_packet.get('hand') != self.target_hand:
                return

            joints = data_packet.get('joints')
            if not joints or not isinstance(joints, list):
                return

            current_time = rospy.Time.now()
            
            # --- 关键修改 1: 手腕位姿来自 ID 1, 而不是 ID 0 ---
            wrist_data = next((joint for joint in joints if joint.get('id') == 1), None)
            
            if wrist_data:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = current_time
                pose_msg.header.frame_id = "pico_world"
                pose_msg.pose.position.x = wrist_data.get('posX', 0.0)
                pose_msg.pose.position.y = wrist_data.get('posY', 0.0)
                pose_msg.pose.position.z = wrist_data.get('posZ', 0.0)
                pose_msg.pose.orientation.x = wrist_data.get('rotX', 0.0)
                pose_msg.pose.orientation.y = wrist_data.get('rotY', 0.0)
                pose_msg.pose.orientation.z = wrist_data.get('rotZ', 0.0)
                pose_msg.pose.orientation.w = wrist_data.get('rotW', 1.0)
                self.pose_pub.publish(pose_msg)

            # --- 关键修改 2: 发布完整的、排序后的26个点 ---
            if len(joints) >= 26:
                hand_msg = PicoHand()
                hand_msg.header.stamp = current_time
                
                # 按ID排序，确保数组索引与PICO ID完全对应
                joints.sort(key=lambda j: j.get('id', 99))
                
                # 将所有26个关节点的位置都打包进去
                keypoints_list = [Point(p.get('posX',0), p.get('posY',0), p.get('posZ',0)) for p in joints]
                hand_msg.keypoints = keypoints_list
                self.keypoints_pub.publish(hand_msg)

                rospy.loginfo_throttle(1.0, f"--> SUCCESS: Published data for '{self.target_hand}' with {len(joints)} joints.")

        except (json.JSONDecodeError, KeyError, IndexError) as e:
            rospy.logwarn_throttle(2.0, f"Failed to process packet: {e}. Packet snippet: '{json_str[:50]}...'")

if __name__ == '__main__':
    try:
        provider = PicoJsonDataProvider()
        provider.run_publisher_loop()
    except rospy.ROSInterruptException:
        rospy.loginfo("PICO Data Provider shutting down.")