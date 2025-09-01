#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose
import numpy as np
from teleop_nodes.msg import UnifiedDataPacket

class TeleopControlNode:
    def __init__(self):
        rospy.init_node('teleop_control_node', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.hand_group = moveit_commander.MoveGroupCommander("hand")

        self.arm_group.set_planning_time(0.5)
        self.hand_group.set_planning_time(0.5)

        self.hand_joint_names = self.hand_group.get_active_joints()
        self.hand_joint_bounds = {name: self.robot.get_joint(name).bounds() for name in self.hand_joint_names}
        rospy.loginfo("Hand joint bounds successfully loaded.")

        self.unified_data_sub = rospy.Subscriber('/pico/unified_data', UnifiedDataPacket, self.unified_data_callback, queue_size=1)

        self.latest_pico_data = None
        self.rate = rospy.Rate(10) # 10Hz控制频率

        # 安全工作区中心点
        self.safe_center_pos = np.array([0.0539, 0.1435, 0.7762])
        self.workspace_half_size = np.array([0.1, 0.15, 0.1])

        rospy.loginfo("Teleop Control Node: Initialized. Waiting for PICO data...")

    def unified_data_callback(self, data_msg):
        self.latest_pico_data = data_msg

    def run(self):
        """
        主运行循环：使用“离合器”机制进行控制。
        """
        while not rospy.is_shutdown():
            if self.latest_pico_data:
                
                # 获取扳机键作为“离合器”
                trigger_value = self.latest_pico_data.trigger_val
                TRIGGER_THRESHOLD = 0.5 # 扣下扳机超过一半视为激活

                # -------------------------------------------------------------
                # (关键) 实现你设计的“离合器”状态机
                # -------------------------------------------------------------
                if trigger_value > TRIGGER_THRESHOLD:
                    # **模式一：手臂控制激活**
                    # 当用户扣下扳机时，手臂跟随手柄。
                    rospy.loginfo_throttle(1, "MODE: ARM CONTROL (Trigger pressed)")
                    self.control_arm(self.latest_pico_data.controller_pos)
                else:
                    # **模式二：手部控制激活**
                    # 当用户松开扳机时，手臂保持不动，灵巧手跟随裸手。
                    rospy.loginfo_throttle(1, "MODE: HAND CONTROL (Trigger released)")
                    self.arm_group.stop() # 确保手臂停止运动
                    self.control_hand(self.latest_pico_data.right_hand_joints)

            else:
                rospy.logwarn_throttle(5, "Waiting for first PICO data packet.")
            
            self.rate.sleep()

    def control_arm(self, norm_pos):
        target_pos_np = self.safe_center_pos + np.array([norm_pos.x, norm_pos.y, norm_pos.z]) * self.workspace_half_size
        
        target_pose = Pose()
        target_pose.position.x, target_pose.position.y, target_pose.position.z = target_pos_np
        target_pose.orientation.w = 1.0
        
        try:
            self.arm_group.set_pose_target(target_pose)
            self.arm_group.go(wait=False)
        except moveit_commander.MoveItCommanderException as e:
            rospy.logwarn_throttle(2, f"Arm control failed: {e}")

    def control_hand(self, hand_landmarks):
        # -------------------------------------------------------------
        # (核心) 手指姿态重定向算法
        # 这是一个需要你未来重点打磨的函数。
        # 目前，我们先用一个非常简单的逻辑来验证链路。
        # -------------------------------------------------------------
        
        # 简化逻辑：计算拇指指尖和食指指尖的距离，来控制手的开合
        landmarks_dict = {lm.id: lm for lm in hand_landmarks}
        
        # PICO/MediaPipe索引: thumb_tip (4), index_tip (8)
        if 4 in landmarks_dict and 8 in landmarks_dict:
            thumb_tip = np.array([landmarks_dict[4].pos_x, landmarks_dict[4].pos_y, landmarks_dict[4].pos_z])
            index_tip = np.array([landmarks_dict[8].pos_x, landmarks_dict[8].pos_y, landmarks_dict[8].pos_z])
            
            # 计算两个指尖的距离
            distance = np.linalg.norm(thumb_tip - index_tip)
            
            # 将距离映射到[0, 1]的抓握值 (需要根据实际距离范围进行调整)
            # 假设最大距离是0.15米（手完全张开），最小是0.02米（捏合）
            grasp_value = 1.0 - np.clip((distance - 0.02) / (0.15 - 0.02), 0.0, 1.0)
            rospy.loginfo_throttle(1, f"Hand Control: Tip distance={distance:.3f}, Grasp value={grasp_value:.2f}")

            # --- 用这个抓握值去控制所有关节 ---
            joint_goal = {}
            for joint_name in self.hand_joint_names:
                min_bound, max_bound = self.hand_joint_bounds[joint_name]
                if max_bound > 0.1 and min_bound >= -0.1:
                    target_angle = min_bound + grasp_value * (max_bound - min_bound)
                    joint_goal[joint_name] = np.clip(target_angle, min_bound, max_bound)
            
            if not joint_goal: return

            try:
                self.hand_group.set_joint_value_target(joint_goal)
                plan = self.hand_group.plan()
                if plan[0]:
                    self.hand_group.execute(plan[1], wait=False)
            except moveit_commander.MoveItCommanderException as e:
                rospy.logwarn_throttle(2, f"Hand control failed: {e}")
        else:
            rospy.logwarn_throttle(2, "Thumb or Index tip not found in landmark data.")


    def shutdown_hook(self):
        rospy.loginfo("Teleop Control Node: Shutting down.")
        if self.robot:
            for group_name in self.robot.get_group_names():
                group = moveit_commander.MoveGroupCommander(group_name)
                group.stop()
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        node = TeleopControlNode()
        rospy.on_shutdown(node.shutdown_hook)
        node.run()
    except rospy.ROSInterruptException:
        pass