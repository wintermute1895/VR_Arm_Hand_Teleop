#!/usr/bin/env python3
import rospy
import math
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion
from teleop_nodes.msg import UnifiedDataPacket, JointData

class PicoDataSimulatorNode:
    def __init__(self):
        rospy.init_node('pico_data_simulator_node', anonymous=True)
        self.unified_data_pub = rospy.Publisher('/pico/unified_data', UnifiedDataPacket, queue_size=1)
        self.rate = rospy.Rate(30)
        rospy.loginfo("PICO Data Simulator Node: Starting to publish simulated data.")

    def run(self):
        while not rospy.is_shutdown():
            t = rospy.get_time()

            # 1. 模拟PICO手柄在[-1, 1]单位立方体内的归一化运动信号
            sim_controller_pos = Point()
            sim_controller_pos.x = math.sin(t * 0.5)
            sim_controller_pos.y = math.cos(t * 0.3)
            sim_controller_pos.z = math.sin(t * 0.7)
            
            sim_controller_rot = Quaternion(w=1.0) # 保持姿态不变

            # 2. 模拟PICO按钮状态
            # 让扳机值在0到1之间平滑地周期性变化，模拟抓握和释放
            sim_trigger_val = 0.5 * (math.sin(t * 1.5) + 1)

            # 3. 模拟完整的26点裸手关节点数据
            sim_hand_joints = []
            # 模拟手腕(id=0)的位置，让它稍微滞后于手柄，更真实
            wrist_pos = np.array([
                0.9 * math.sin(t * 0.5 - 0.1),
                0.9 * math.cos(t * 0.3 - 0.1),
                0.9 * math.sin(t * 0.7 - 0.1)
            ])
            
            for i in range(26): # 必须生成0-25号所有关节点
                joint = JointData()
                joint.id = i
                
                # 简单模拟：所有关节点都以手腕为中心，并根据扳机值（抓握程度）向中心收缩
                # 这是一个非常粗糙的模拟，但足以测试数据链路和重定向逻辑
                open_pos = wrist_pos + np.random.uniform(-0.05, 0.05, 3) # 手张开时的随机位置
                closed_pos = wrist_pos # 手握紧时都收缩到手腕点
                
                # 根据扳机值在张开和闭合位置之间插值
                current_pos = (1 - sim_trigger_val) * open_pos + sim_trigger_val * closed_pos
                
                joint.pos_x = current_pos[0]
                joint.pos_y = current_pos[1]
                joint.pos_z = current_pos[2]
                joint.rot_w = 1.0 # 姿态保持不变
                sim_hand_joints.append(joint)

            # 4. 填充并发布消息
            unified_msg = UnifiedDataPacket()
            unified_msg.header.stamp = rospy.Time.now()
            unified_msg.controller_pos = sim_controller_pos
            unified_msg.controller_rot = sim_controller_rot
            unified_msg.trigger_val = sim_trigger_val
            unified_msg.right_hand_joints = sim_hand_joints
            
            self.unified_data_pub.publish(unified_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = PicoDataSimulatorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass