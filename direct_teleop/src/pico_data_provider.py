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

        # 将IP和端口保存为实例变量
        self.udp_ip = udp_ip
        self.udp_port = udp_port

        self.pose_pub = rospy.Publisher('/pico/hand/pose', PoseStamped, queue_size=1)
        self.keypoints_pub = rospy.Publisher('/pico/hand/keypoints', PicoHand, queue_size=1)
        
        self.latest_complete_message = None
        self.lock = threading.Lock()
        self.buffer = ""

        self.udp_thread = threading.Thread(target=self._udp_receiver_loop)
        self.udp_thread.daemon = True
        self.udp_thread.start()

        rospy.loginfo(f"PICO JSON Data Provider started. Listening on UDP port {self.udp_port}...")

    def _udp_receiver_loop(self):
        # 这个函数在独立的线程中运行
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 正确使用 self.udp_ip 和 self.udp_port
        sock.bind((self.udp_ip, self.udp_port))
        
        while not rospy.is_shutdown():
            try:
                data, addr = sock.recvfrom(8192)
                with self.lock:
                    self.buffer += data.decode('utf-8', errors='ignore')
            except Exception as e:
                rospy.logerr(f"UDP receiver error: {e}")

    def run_publisher_loop(self):
        rate = rospy.Rate(50) 
        while not rospy.is_shutdown():
            self.process_buffer()
            rate.sleep()

    def process_buffer(self):
        message_to_process = None
        with self.lock:
            if '\n' in self.buffer:
                *_, latest_msg = self.buffer.strip().split('\n')
                self.buffer = ""
                if latest_msg:
                    message_to_process = latest_msg
        
        if message_to_process:
            self.publish_data(message_to_process)

    def publish_data(self, json_str):
        try:
            data_packet = json.loads(json_str)
            if 'joints' not in data_packet or not isinstance(data_packet['joints'], list) or not data_packet['joints']:
                return

            joints = data_packet['joints']
            current_time = rospy.Time.now()
            
            wrist_data = next((joint for joint in joints if joint.get('id') == 0), None)
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

            if len(joints) >= 22:
                hand_msg = PicoHand()
                hand_msg.header.stamp = current_time
                joints.sort(key=lambda j: j.get('id', 99))
                points_for_retargeting = joints[0:22]
                
                keypoints_list = [Point(p.get('posX',0), p.get('posY',0), p.get('posZ',0)) for p in points_for_retargeting]
                hand_msg.keypoints = keypoints_list
                self.keypoints_pub.publish(hand_msg)

        except (json.JSONDecodeError, KeyError, IndexError):
            pass

if __name__ == '__main__':
    try:
        provider = PicoJsonDataProvider()
        provider.run_publisher_loop()
    except rospy.ROSInterruptException:
        pass