#!/usr/bin/env python3
import rospy
import sys
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trac_ik_python.trac_ik import IK

class VrDirectPublisherController:
    """
    一个实时的、基于位置的笛卡尔伺服控制器。
    它接收目标位姿，通过IK解算，并以安全的方式直接向硬件的
    ros_control position_controllers发布关节角度指令。
    """
    def __init__(self):
        rospy.init_node('vr_direct_publisher_controller')

        # --- 1. 核心参数 ---
        self.base_link = "base_link"
        self.end_effector_link = "Link7"
        self.ik_timeout = 0.01  # IK求解超时时间 (s)
        self.control_rate = 100 # 主控制循环频率 (Hz)

        # --- 2. 安全与平滑参数 (非常重要！) ---
        self.max_joint_velocity = 0.8  # 单个关节最大速度 (rad/s) - 请根据实际情况调整
        self.max_joint_acceleration = 5.0 # 单个关节最大加速度 (rad/s^2) - 用于平滑启动和停止

        # --- 3. 加载URDF并初始化IK ---
        try:
            self.urdf_string = rospy.get_param("/robot_description")
            if len(self.urdf_string) == 0:
                raise rospy.ROSException("URDF is empty!")
            
            self.ik_solver = IK(self.base_link, self.end_effector_link,
                                urdf_string=self.urdf_string, timeout=self.ik_timeout)
            
            self.joint_names = self.ik_solver.joint_names
            rospy.loginfo(f"TRAC-IK solver initialized for joints: {self.joint_names}")

        except Exception as e:
            rospy.logerr(f"Failed to initialize controller: {e}")
            sys.exit(1)

        # --- 4. 状态变量 ---
        self.last_commanded_positions = None
        self.last_commanded_velocity = np.zeros(len(self.joint_names))
        self.latest_target_pose = None
        self.last_joint_state = None
        self.control_active = False

        # --- 5. ROS 接口 ---
        self.publishers = [rospy.Publisher(f'/arm/{name}_position_controller/command', Float64, queue_size=1) for name in self.joint_names]
        
        rospy.Subscriber('/pico/hand/pose', PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback, queue_size=1)

        rospy.loginfo("VR Direct Publisher Controller is ready.")

    def pose_callback(self, msg):
        """接收到新的PICO位姿时，更新目标"""
        self.latest_target_pose = msg.pose
        if not self.control_active:
            self.control_active = True
            rospy.loginfo("First PICO pose received. Starting control loop.")

    def joint_states_callback(self, msg):
        """接收机器人当前真实的关节状态"""
        self.last_joint_state = msg

    def get_current_positions(self):
        """从/joint_states消息中安全地提取当前关节角度"""
        if self.last_joint_state is None:
            return None
        
        positions = []
        try:
            for name in self.joint_names:
                idx = self.last_joint_state.name.index(name)
                positions.append(self.last_joint_state.position[idx])
            return np.array(positions)
        except (ValueError, AttributeError):
            return None

    def run(self):
        """主控制循环"""
        rate = rospy.Rate(self.control_rate)
        dt = 1.0 / self.control_rate

        while not rospy.is_shutdown():
            if not self.control_active or self.latest_target_pose is None:
                rate.sleep()
                continue
            
            current_positions = self.get_current_positions()
            if current_positions is None:
                rospy.logwarn_throttle(1.0, "Waiting for valid joint states...")
                rate.sleep()
                continue

            # 如果是第一次循环，用当前位置初始化
            if self.last_commanded_positions is None:
                self.last_commanded_positions = current_positions

            # --- IK 解算 ---
            target_ik_solution = self.ik_solver.get_ik(
                current_positions, # 使用真实关节位置作为种子
                self.latest_target_pose.position.x, self.latest_target_pose.position.y, self.latest_target_pose.position.z,
                self.latest_target_pose.orientation.x, self.latest_target_pose.orientation.y, self.latest_target_pose.orientation.z, self.latest_target_pose.orientation.w
            )

            if target_ik_solution:
                target_positions = np.array(target_ik_solution)
                
                # --- 安全平滑处理 ---
                # 1. 计算目标速度
                target_velocity = (target_positions - self.last_commanded_positions) / dt
                
                # 2. 加速度限制
                accel = (target_velocity - self.last_commanded_velocity) / dt
                accel_limited = np.clip(accel, -self.max_joint_acceleration, self.max_joint_acceleration)
                
                # 3. 根据限制后的加速度计算新的目标速度
                velocity_after_accel_limit = self.last_commanded_velocity + accel_limited * dt
                
                # 4. 速度限制
                velocity_limited = np.clip(velocity_after_accel_limit, -self.max_joint_velocity, self.max_joint_velocity)

                # 5. 根据最终的速度计算这一步要移动到的最终位置
                final_target_positions = self.last_commanded_positions + velocity_limited * dt
                
                # --- 发布指令 ---
                for i, pos in enumerate(final_target_positions):
                    self.publishers[i].publish(Float64(pos))
                
                # --- 更新状态用于下一轮循环 ---
                self.last_commanded_positions = final_target_positions
                self.last_commanded_velocity = velocity_limited

            else:
                rospy.logwarn_throttle(1.0, "IK solution not found. Holding last position.")
                # 如果IK无解，重复发布上一个有效指令以保持姿态
                for i, pos in enumerate(self.last_commanded_positions):
                    self.publishers[i].publish(Float64(pos))

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = VrDirectPublisherController()
        controller.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("Controller shutting down.")