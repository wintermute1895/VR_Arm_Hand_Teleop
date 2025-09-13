#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trac_ik_python.trac_ik import IK

class VrRvizController:
    """
    这个节点负责将VR手柄的位姿数据通过逆运动学(IK)解算，
    转换为机器人的关节角度，并发布到/joint_states话题，
    从而在RViz中实时地驱动机器人模型进行可视化。
    """
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('vr_rviz_controller')

        # --- 1. 定义核心参数 ---
        # 机械臂的基座Link名称 (IK求解的参考坐标系)
        self.base_link = "base_link" 
        # 机械臂的末端Link名称 (IK求解的目标Link)
        self.end_effector_link = "Link6" 
        # IK求解的超时时间 (秒)
        self.ik_timeout = 0.005 
        # 发布频率 (Hz)
        self.publish_rate = 50 

        # --- 2. 加载URDF模型 ---
        rospy.loginfo("Loading URDF from parameter server '/robot_description'...")
        try:
            self.urdf_string = rospy.get_param("/robot_description")
            if len(self.urdf_string) == 0:
                raise rospy.ROSException("URDF is empty, is the model loaded?")
            rospy.loginfo(f"URDF loaded successfully (length: {len(self.urdf_string)} characters).")
        except (rospy.ROSException, KeyError) as e:
            rospy.logerr(f"Failed to load URDF from '/robot_description'. Error: {e}")
            rospy.logerr("Please ensure you have a launch file running that loads the robot model.")
            sys.exit(1) # 严重错误，直接退出

        # --- 3. 初始化IK求解器 ---
        try:
            self.ik_solver = IK(self.base_link, 
                                self.end_effector_link, 
                                urdf_string=self.urdf_string, 
                                timeout=self.ik_timeout)
            rospy.loginfo("TRAC-IK solver initialized successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize TRAC-IK solver. Error: {e}")
            sys.exit(1)

        # --- 4. 初始化状态变量 ---
        # 存储最新的目标关节角度，初始化为7个0
        self.target_joint_angles = [0.0] * self.ik_solver.number_of_joints
        
        # 存储IK求解器期望的关节名称顺序
        self.joint_names = self.ik_solver.joint_names

        # --- 5. 设置ROS订阅与发布 ---
        # 订阅PICO手柄发布的位姿数据
        rospy.Subscriber('/pico/hand_pose', PoseStamped, self.pico_pose_callback, queue_size=1)
        
        # 发布/joint_states，供robot_state_publisher和RViz使用
        self.joint_states_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # 创建一个定时器，以固定频率调用publish_joint_states函数
        self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_joint_states)
        
        rospy.loginfo("VR RViz Controller is ready and waiting for PICO data on /pico/hand_pose...")

    def pico_pose_callback(self, msg):
        """
        接收到新的PICO位姿时被调用
        """
        pose = msg.pose
        
        # 使用上一次成功解算的角度作为IK的“种子”，这能让解算更快、更稳定
        seed_state = self.target_joint_angles
        
        # 调用IK求解器
        new_target_joints = self.ik_solver.get_ik(seed_state,
                                                  pose.position.x, pose.position.y, pose.position.z,
                                                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

        if new_target_joints:
            # 如果解算成功，更新目标关节角度
            self.target_joint_angles = new_target_joints
        else:
            # 如果解算失败（例如，目标位置在机械臂工作范围外），打印警告并保持上一个姿态
            rospy.logwarn_throttle(1.0, "IK solution not found. Holding the last valid position.")

    def publish_joint_states(self, event):
        """
        由定时器以固定频率调用，持续发布当前的关节状态
        """
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        js_msg.name = self.joint_names 
        js_msg.position = self.target_joint_angles
        js_msg.velocity = []  # 可选
        js_msg.effort = []    # 可选
        
        self.joint_states_pub.publish(js_msg)
            
    def run(self):
        """
        保持节点运行
        """
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = VrRvizController()
        controller.run()
    except rospy.ROSInterruptException:
        pass