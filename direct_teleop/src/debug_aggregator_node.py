#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool # 导入布尔消息
from direct_teleop.msg import DebugInfo, LossInfo # 导入我们所有的自定义消息

class DebugAggregatorNode:
    def __init__(self):
        rospy.init_node('debug_aggregator_node')

        # --- 1. 初始化数据存储 ---
        self.latest_data = {
            'arm_target_pos_x': 0.0,
            'arm_target_pos_y': 0.0,
            'arm_target_pos_z': 0.0,
            'ik_solution_found': False,
            'total_loss': 0.0,
            'loss_align': 0.0,
            'loss_couple': 0.0,
            'loss_smooth': 0.0,
        }

        # --- 2. 创建发布器 ---
        self.debug_pub = rospy.Publisher('/teleop/debug_info', DebugInfo, queue_size=10)

        # --- 3. 创建订阅者 (现在是完整的了！) ---
        rospy.Subscriber('/teleop/target_pose', PoseStamped, self.arm_target_callback)
        rospy.Subscriber('/debug/ik_status', Bool, self.ik_status_callback)
        rospy.Subscriber('/debug/loss_info', LossInfo, self.loss_info_callback)

        # --- 4. 启动定时发布循环 ---
        self.publish_rate = 20 # Hz
        rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_loop)

        rospy.loginfo("Debug Aggregator Node is running and subscribed to all debug topics.")

    def arm_target_callback(self, msg):
        """存储最新的手臂目标位置"""
        self.latest_data['arm_target_pos_x'] = msg.pose.position.x
        self.latest_data['arm_target_pos_y'] = msg.pose.position.y
        self.latest_data['arm_target_pos_z'] = msg.pose.position.z

    def ik_status_callback(self, msg):
        """存储最新的IK求解状态"""
        self.latest_data['ik_solution_found'] = msg.data

    def loss_info_callback(self, msg):
        """存储最新的手部重定向损失值"""
        self.latest_data['total_loss'] = msg.total_loss
        self.latest_data['loss_align'] = msg.loss_align
        self.latest_data['loss_couple'] = msg.loss_couple
        self.latest_data['loss_smooth'] = msg.loss_smooth

    def publish_loop(self, event):
        """定时循环：打包并发布DebugInfo消息"""
        debug_msg = DebugInfo()
        debug_msg.header.stamp = rospy.Time.now()

        # 从存储中填充消息的所有字段
        debug_msg.arm_target_pos_x = self.latest_data['arm_target_pos_x']
        debug_msg.arm_target_pos_y = self.latest_data['arm_target_pos_y']
        debug_msg.arm_target_pos_z = self.latest_data['arm_target_pos_z']
        debug_msg.ik_solution_found = self.latest_data['ik_solution_found']
        debug_msg.total_loss = self.latest_data['total_loss']
        debug_msg.loss_align = self.latest_data['loss_align']
        debug_msg.loss_couple = self.latest_data['loss_couple']
        debug_msg.loss_smooth = self.latest_data['loss_smooth']
        
        self.debug_pub.publish(debug_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = DebugAggregatorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass