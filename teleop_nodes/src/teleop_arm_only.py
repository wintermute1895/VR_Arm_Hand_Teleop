#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# rosrun teleop_nodes teleop_arm_only.py

"""
teleop_arm_only.py (FINAL-FIXED Version)

- Fixes all known errors, including the Python f-string formatting TypeError.
- This version should be stable and allow you to focus on calibration.
"""

import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from moveit_msgs.msg import RobotTrajectory
import numpy as np
from teleop_nodes.msg import UnifiedDataPacket
from tf.transformations import quaternion_multiply, quaternion_from_euler

class TeleopArmOnlyNode:
    def __init__(self):
        rospy.init_node('teleop_arm_only_node', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()

        try:
            self.arm_group = moveit_commander.MoveGroupCommander("arm")
        except RuntimeError as e:
            rospy.logerr(f"CRITICAL ERROR: Failed to initialize 'arm' MoveGroup. Error: {e}")
            sys.exit(1)
            
        self.pico_data_sub = rospy.Subscriber(
            '/pico/unified_data', 
            UnifiedDataPacket, 
            self.pico_data_callback, 
            queue_size=1, 
            tcp_nodelay=True
        )
        self.marker_pub = rospy.Publisher('/teleop_target_marker', Marker, queue_size=10)

        # --- 核心校准参数 ---
        # **调试点1**: 将你的手放在舒适的起始位置，运行一次，观察终端打印的[RAW PICO DATA] Pos值，
        # 然后把那个值填到这里。
        self.pico_origin_offset = np.array([0.15, 1.60, -0.20]) # <--- 在这里填入你记录的“手部零点”

        # **调试点2**: 机器人工作区中心
        self.robot_workspace_center = np.array([0.5, 0.0, 0.4])
        # **调试点3**: 缩放比例
        self.pico_input_scale = np.array([1.0, 1.0, 1.0])
        # **调试点4**: 姿态校准
        self.pico_to_robot_quat_offset = quaternion_from_euler(
            np.radians(-90), 
            np.radians(0), 
            np.radians(90)
        )
        
        self.WRIST_JOINT_ID = 0
        
        rospy.loginfo("Teleop Arm-Only Node: Initialized.")
        rospy.loginfo(">>> To debug, ADD a Marker display in RViz on topic '/teleop_target_marker' <<<")

    def pico_data_callback(self, data_msg):
        if not data_msg.right_hand_joints:
            return
        self.control_arm(data_msg.right_hand_joints)

    def control_arm(self, all_joints_data):
        wrist_data = next((joint for joint in all_joints_data if joint.id == self.WRIST_JOINT_ID), None)
        if wrist_data is None: return

        pico_pos_raw = np.array([wrist_data.pos_x, wrist_data.pos_y, wrist_data.pos_z])
        pico_quat = np.array([wrist_data.rot_x, wrist_data.rot_y, wrist_data.rot_z, wrist_data.rot_w])

        pico_pos_relative = pico_pos_raw - self.pico_origin_offset
        
        # --- 核心修复：移除了对NumPy数组的非法格式化 ---
        rospy.loginfo_throttle(1, f"Relative PICO Pos: {pico_pos_relative}")

        robot_pos_relative = np.array([pico_pos_relative[2], -pico_pos_relative[0], pico_pos_relative[1]])

        target_pos = self.robot_workspace_center + robot_pos_relative * self.pico_input_scale
        rospy.loginfo_throttle(1, f"Final Target Pos: {target_pos}")
        
        target_quat = quaternion_multiply(pico_quat, self.pico_to_robot_quat_offset)

        target_pose = Pose()
        target_pose.position.x, target_pose.position.y, target_pose.position.z = target_pos
        target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w = target_quat

        self.publish_target_marker(target_pose)

        self.arm_group.set_goal_position_tolerance(0.05)
        self.arm_group.set_goal_orientation_tolerance(0.1)
        self.arm_group.set_planning_time(0.5)
        self.arm_group.set_num_planning_attempts(5)

        plan_result = self.arm_group.plan(target_pose)
        
        if isinstance(plan_result, RobotTrajectory) and plan_result.joint_trajectory.points:
            rospy.loginfo_throttle(1, "Planning successful!")
            self.arm_group.execute(plan_result, wait=False)
        else:
            rospy.logwarn_throttle(1, "Motion plan FAILED. Check the RED ARROW's position and orientation in RViz.")

    def publish_target_marker(self, pose):
        marker = Marker()
        marker.header.frame_id = self.robot.get_planning_frame()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "teleop_target_pose"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x, marker.scale.y, marker.scale.z = 0.15, 0.03, 0.03
        marker.color.a = 0.9
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

    def shutdown_hook(self):
        rospy.loginfo("Teleop Arm-Only Node: Shutting down.")
        if self.arm_group:
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TeleopArmOnlyNode()
        rospy.on_shutdown(node.shutdown_hook)
        node.run()
    except rospy.ROSInterruptException:
        pass