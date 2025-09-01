#!/usr/bin/env python3
import rospy
import socket
import json
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion
from teleop_nodes.msg import UnifiedDataPacket, JointData

class PicoRealBridgeNode:
    def __init__(self):
        rospy.init_node('pico_real_bridge_node', anonymous=True)

        # (关键) 获取参数，而不是硬编码
        self.listen_ip = rospy.get_param("~listen_ip", "0.0.0.0")
        self.listen_port = rospy.get_param("~listen_port", 9999)

        # UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.listen_ip, self.listen_port))
        rospy.loginfo(f"PICO Real Bridge: Listening for UDP on {self.listen_ip}:{self.listen_port}")

        # ROS Publisher
        self.unified_data_pub = rospy.Publisher('/pico/unified_data', UnifiedDataPacket, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            try:
                data, addr = self.sock.recvfrom(65535) # 接收数据
                json_str = data.decode('utf-8')
                parsed_data = json.loads(json_str) # 解析JSON
                
                # (关键) 将收到的Python字典，转换成ROS消息
                unified_msg = self.dict_to_ros_msg(parsed_data)
                
                self.unified_data_pub.publish(unified_msg) # 发布ROS消息
            except Exception as e:
                rospy.logwarn_throttle(5, f"PICO Real Bridge: Error processing UDP packet: {e}")

    def dict_to_ros_msg(self, data_dict):
        msg = UnifiedDataPacket()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "pico_world"

        # 手柄位姿
        msg.controller_pos = Point(**data_dict['controllerPos'])
        msg.controller_rot = Quaternion(**data_dict['controllerRot'])

        # 按钮状态
        msg.primary_btn = data_dict['primaryBtn']
        msg.secondary_btn = data_dict['secondaryBtn']
        msg.trigger_val = data_dict['triggerVal']
        msg.grip_val = data_dict['gripVal']
        msg.axis_vec2_x = data_dict['axisVec2X']
        msg.axis_vec2_y = data_dict['axisVec2Y']
        msg.menu_btn = data_dict['menuBtn']

        # 裸手关节
        for joint_dict in data_dict.get('rightHandJoints', []):
            joint_msg = JointData()
            joint_msg.id = joint_dict['id']
            joint_msg.pos_x = joint_dict['posX']
            joint_msg.pos_y = joint_dict['posY']
            joint_msg.pos_z = joint_dict['posZ']
            joint_msg.rot_x = joint_dict['rotX']
            joint_msg.rot_y = joint_dict['rotY']
            joint_msg.rot_z = joint_dict['rotZ']
            joint_msg.rot_w = joint_dict['rotW']
            msg.right_hand_joints.append(joint_msg)
        
        return msg

    def shutdown_hook(self):
        rospy.loginfo("PICO Real Bridge: Shutting down UDP socket.")
        self.sock.close()

if __name__ == '__main__':
    node = PicoRealBridgeNode()
    rospy.on_shutdown(node.shutdown_hook)
    node.run()