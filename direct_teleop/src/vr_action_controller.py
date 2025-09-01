#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from rm_msgs.msg import JointPos
from trac_ik_python.trac_ik import IK
from tf.transformations import quaternion_matrix, quaternion_from_matrix

# ... (辅助函数 pose_to_matrix 和 matrix_to_pose 保持不变) ...
def pose_to_matrix(pose):
    q = pose.orientation
    pos = pose.position
    mat = quaternion_matrix([q.x, q.y, q.z, q.w])
    mat[0:3, 3] = [pos.x, pos.y, pos.z]
    return mat

def matrix_to_pose(mat):
    pose = Pose()
    q = quaternion_from_matrix(mat)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    pose.position.x, pose.position.y, pose.position.z = mat[0:3, 3]
    return pose
# ==============================================================================

class VrDirectTransparentController:
    def __init__(self):
        rospy.init_node('vr_transparent_controller')

        # --- 1. 核心参数 ---
        self.base_link = "base_link"
        self.end_effector_link = "Link6"
        self.control_rate = 50
        self.ik_timeout = 0.005

        # --- 2. 安全与平滑参数 ---
        self.max_joint_velocity = 0.2
        self.max_joint_acceleration = 1.0

        # --- 3. 标定矩阵 ---
        self.T_pico_to_robot = np.array([
            [-0.85111942, 0.18628482, -0.49080924, -0.10282520],
            [0.07385891, 0.96811631, 0.23936513, -1.19817663],
            [0.51975052, 0.16747767, -0.83774138, 0.66056851],
            [0.00000000, 0.00000000, 0.00000000, 1.00000000],
        ])

        # --- 4. 初始化IK ---
        try:
            rospy.loginfo("Waiting for '/robot_description' parameter...")
            self.urdf_string = rospy.get_param("/robot_description")
            self.ik_solver = IK(self.base_link, self.end_effector_link, urdf_string=self.urdf_string, timeout=self.ik_timeout)
            self.joint_names = self.ik_solver.joint_names
            if len(self.joint_names) != 6:
                rospy.logwarn(f"IK solver initialized for {len(self.joint_names)} joints, but expected 6 for RM-65.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize: {e}")
            sys.exit(1)

        # --- 5. 状态变量 ---
        self.last_commanded_positions = None
        self.last_commanded_velocity = np.zeros(len(self.joint_names))
        self.latest_target_pose = None
        self.control_active = False

        # --- 6. ROS 接口 ---
        self.command_pub = rospy.Publisher('/rm_driver/JointPos', JointPos, queue_size=1)
        rospy.Subscriber('/pico/hand/pose', PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback, queue_size=1)
        
        rospy.loginfo("VR Direct Transparent Controller is ready.")

    def joint_states_callback(self, msg):
        if self.last_commanded_positions is None:
            try:
                current_positions = [msg.position[msg.name.index(name)] for name in self.joint_names]
                if len(current_positions) == len(self.joint_names):
                    self.last_commanded_positions = np.array(current_positions)
                    rospy.loginfo(f"Controller initialized with joint states: {[f'{q:.3f}' for q in self.last_commanded_positions]}")
            except ValueError:
                pass
    
    def pose_callback(self, msg):
        self.latest_target_pose = msg.pose
        if not self.control_active and self.last_commanded_positions is not None:
            self.control_active = True
            rospy.loginfo("First PICO pose received. Starting control loop.")

    def run(self):
        rate = rospy.Rate(self.control_rate)
        dt = 1.0 / self.control_rate

        while not rospy.is_shutdown():
            if not self.control_active or self.latest_target_pose is None:
                rate.sleep()
                continue
            
            mat_pico = pose_to_matrix(self.latest_target_pose)
            mat_robot_target = self.T_pico_to_robot @ mat_pico
            target_pose = matrix_to_pose(mat_robot_target)

            # 2. IK 解算
            target_ik_solution = self.ik_solver.get_ik(
                list(self.last_commanded_positions),
                target_pose.position.x, target_pose.position.y, target_pose.position.z,
                target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w,
                # !! 关键修改：增加容忍度，让IK更容易找到解 !!
                bx=0.05, by=0.05, bz=0.05, # 位置容忍度 (5cm)
                brx=0.3, bry=0.3, brz=0.3    # 姿态容忍度 (~17度)
            )

            if target_ik_solution:
                target_positions = np.array(target_ik_solution)
                
                # 3. 安全平滑处理...
                target_velocity = (target_positions - self.last_commanded_positions) / dt
                accel = (target_velocity - self.last_commanded_velocity) / dt
                accel_limited = np.clip(accel, -self.max_joint_acceleration, self.max_joint_acceleration)
                velocity_after_accel_limit = self.last_commanded_velocity + accel_limited * dt
                velocity_limited = np.clip(velocity_after_accel_limit, -self.max_joint_velocity, self.max_joint_velocity)
                final_target_positions = self.last_commanded_positions + velocity_limited * dt
                
                # 4. 发布透传指令...
                cmd_msg = JointPos()
                cmd_msg.joint = list(final_target_positions)
                self.command_pub.publish(cmd_msg)
                
                self.last_commanded_positions = final_target_positions
                self.last_commanded_velocity = velocity_limited
            else:
                rospy.logwarn_throttle(1.0, "IK solution not found. Holding last position.")
                cmd_msg = JointPos()
                cmd_msg.joint = list(self.last_commanded_positions)
                self.command_pub.publish(cmd_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = VrDirectTransparentController()
        controller.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("Controller shutting down.")