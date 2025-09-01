#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from direct_teleop.msg import PicoHand  # 导入我们自定义的消息
from tf.transformations import quaternion_from_euler

class PicoMasterSimulator:
    def __init__(self):
        rospy.init_node('pico_master_simulator')

        # === 发布器 ===
        self.pose_pub = rospy.Publisher('/pico/hand/pose', PoseStamped, queue_size=10)
        self.keypoints_pub = rospy.Publisher('/pico/hand/keypoints', PicoHand, queue_size=10)

        self.rate = rospy.Rate(50)  # 50Hz

        # === 手臂运动参数 ===
        self.center_x, self.center_y, self.center_z = 0.35, 0.0, 0.4
        self.radius = 0.05

        # === 手指运动参数 ===
        # 加载一个静态的、张开手掌的模板 (22个点)
        self.open_hand_template = self._get_hand_template()
        self.time_step = 0.0

        rospy.loginfo("PICO Master Simulator started.")

    def _get_hand_template(self):
        # 这是一个简化的、表示张开手掌的模板关键点数据 (相对于手腕)
        # 在真实应用中，这将由PICO SDK提供
        # (单位: 米)
        return np.array([
            [0.0, 0.0, 0.0],  # wrist
            [0.0, -0.02, 0.03], # thumb_mcp
            [0.0, -0.04, 0.05], # thumb_pip
            [0.0, -0.06, 0.06], # thumb_dip
            [0.0, -0.08, 0.07], # thumb_tip
            [-0.04, 0.0, 0.08], # index_mcp
            [-0.04, 0.0, 0.12], # index_pip
            [-0.04, 0.0, 0.15], # index_dip
            [-0.04, 0.0, 0.18], # index_tip
            [0.0, 0.0, 0.09],   # middle_mcp
            [0.0, 0.0, 0.13],   # middle_pip
            [0.0, 0.0, 0.16],   # middle_dip
            [0.0, 0.0, 0.19],   # middle_tip
            [0.04, 0.0, 0.08],  # ring_mcp
            [0.04, 0.0, 0.12],  # ring_pip
            [0.04, 0.0, 0.15],  # ring_dip
            [0.04, 0.0, 0.18],  # ring_tip
            [0.08, 0.0, 0.07],  # little_mcp
            [0.08, 0.0, 0.10],  # little_pip
            [0.08, 0.0, 0.12],  # little_dip
            [0.08, 0.0, 0.14],  # little_tip
            [0.0, 0.0, 0.05]    # palm_center
        ])

    def run(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            # --- 1. 发布手臂位姿 (画圆) ---
            pose_msg = PoseStamped()
            pose_msg.header.stamp = current_time
            pose_msg.header.frame_id = 'base_link'
            
            pose_msg.pose.position.x = self.center_x
            pose_msg.pose.position.y = self.center_y + self.radius * math.cos(self.time_step)
            pose_msg.pose.position.z = self.center_z + self.radius * math.sin(self.time_step)
            
            q = quaternion_from_euler(0, math.pi / 2, 0)
            pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = q
            
            self.pose_pub.publish(pose_msg)

            # --- 2. 发布手部关键点 (开合) ---
            hand_msg = PicoHand()
            hand_msg.header.stamp = current_time
            
            # 模拟手指开合：通过一个缩放因子
            scale = 0.8 + 0.2 * math.sin(self.time_step * 2) # 在0.8到1.0之间变化
            
            # 应用缩放并转换为Point[]
            scaled_points = self.open_hand_template * scale
            hand_msg.keypoints = [Point(x, y, z) for x, y, z in scaled_points]

            self.keypoints_pub.publish(hand_msg)

            # --- 更新和休眠 ---
            self.time_step += 0.05
            self.rate.sleep()

if __name__ == '__main__':
    try:
        simulator = PicoMasterSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass