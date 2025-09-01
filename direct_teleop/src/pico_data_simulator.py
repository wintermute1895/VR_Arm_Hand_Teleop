#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class PicoDataSimulator:
    def __init__(self):
        """
        初始化PICO模拟数据发布节点
        """
        # 初始化ROS节点
        rospy.init_node('pico_data_simulator')

        # 创建一个Publisher，发布到/pico/hand_pose话题，消息类型为PoseStamped
        self.pose_pub = rospy.Publisher('/pico/hand_pose', PoseStamped, queue_size=10)

        # 设置发布频率为50Hz，模拟VR设备的数据率
        self.rate = rospy.Rate(50)

        # 定义运动的起始中心点 (单位：米)
        # 这个位置大概在机械臂正前方稍靠上的位置
        self.center_x = 0.35
        self.center_y = 0.0
        self.center_z = 0.4

        # 定义运动轨迹的半径
        self.radius = 0.05

        # 定义姿态旋转的速度
        self.rotation_speed = 0.

        # 时间步，用于生成周期性运动
        self.time_step = 0.0

        rospy.loginfo("PICO Data Simulator started. Publishing to /pico/hand_pose...")

    def run(self):
        """
        主循环，用于计算并发布模拟的位姿数据
        """
        while not rospy.is_shutdown():
            # 1. 创建一个 PoseStamped 消息实例
            pose_msg = PoseStamped()

            # 2. 填充消息头 (Header)
            #    时间戳使用当前ROS时间，这对于很多ROS工具至关重要
            pose_msg.header.stamp = rospy.Time.now()
            #    坐标系ID设为'base_link'，告诉ROS这个位姿是相对于机器人基座的
            pose_msg.header.frame_id = 'base_link'

            # 3. 计算模拟的位置 (Position)
            #    让手柄在Y-Z平面上画一个圆周运动
            pose_msg.pose.position.x = self.center_x
            pose_msg.pose.position.y = self.center_y + self.radius * math.cos(self.time_step)
            pose_msg.pose.position.z = self.center_z + self.radius * math.sin(self.time_step)

            # 4. 计算模拟的姿态 (Orientation)
            #    让手柄的姿态也随着时间周期性旋转 (绕Z轴，即yaw角)
            yaw_angle = self.time_step * self.rotation_speed
            q = quaternion_from_euler(0, 0, yaw_angle) # Roll, Pitch, Yaw
            
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]

            # 5. 发布消息
            self.pose_pub.publish(pose_msg)

            # 6. 更新时间步，为下一帧做准备
            self.time_step += 0.05

            # 7. 按照设定的频率休眠
            self.rate.sleep()

if __name__ == '__main__':
    try:
        simulator = PicoDataSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("PICO Data Simulator shut down.")