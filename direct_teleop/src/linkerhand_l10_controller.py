#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

from hand_retargeting_lib.L10.utils_l10 import convert_q_to_l10_command
import hand_retargeting_lib.L10.config_l10 as config

class LinkerHandL10Controller:
    def __init__(self):
        rospy.init_node('linkerhand_l10_controller')

        self.hardware_topic = '/cb_left_hand_control_cmd'
        self.hardware_msg_type = JointState

        self.target_sub = rospy.Subscriber(
            '/hand_retargeting/target_joint_states', 
            JointState, 
            self.target_callback, 
            queue_size=1
        )

        self.command_pub = rospy.Publisher(
            self.hardware_topic, 
            self.hardware_msg_type, 
            queue_size=10
        )
        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("LinkerHand L10 Controller is running and waiting for target joint states...")

    def shutdown_hook(self):
        """
        当节点被关闭时（例如按Ctrl+C），这个函数会被自动调用。
        """
        rospy.loginfo("Shutdown signal received. Resetting hand to open position...")
        
        # 创建一个“张开”指令
        open_pose_cmd = JointState()
        open_pose_cmd.header.stamp = rospy.Time.now()
        # 根据L10 SDK文档的默认张开姿态
        open_pose_cmd.position = [255, 128, 255, 255, 255, 255, 128, 128, 128, 128]

        # 发布指令
        self.command_pub.publish(open_pose_cmd)
        # 短暂等待，确保消息有足够的时间被发送出去
        rospy.sleep(0.1) 
        rospy.loginfo("Hand reset command sent. Node is shutting down.")
        
    def target_callback(self, joint_state_msg):
        rospy.loginfo_throttle(1.0, "--- Received new target joint state! ---")
        try:
            # 1. 将接收到的JointState消息转换为q_dict字典
            q_dict = {}
            for finger in config.FINGER_NAMES:
                q_dict[finger] = [0.0] * len(config.JOINT_INFO[finger]['names'])
            
            for i, name in enumerate(joint_state_msg.name):
                for finger, info in config.JOINT_INFO.items():
                    if name in info['names']:
                        joint_idx = info['names'].index(name)
                        q_dict[finger][joint_idx] = joint_state_msg.position[i]
                        break
            
            rospy.loginfo_throttle(1.0, f"  Step 1: Successfully converted to q_dict.")

            # 2. 调用L10的专用转换函数
            position_cmd_list = convert_q_to_l10_command(q_dict,config)
            
            rospy.loginfo_throttle(1.0, f"  Step 2: Converted to hardware command: {position_cmd_list}")

            # 3. 创建并填充硬件指令消息
            hardware_cmd_msg = JointState()
            hardware_cmd_msg.header.stamp = rospy.Time.now()
            hardware_cmd_msg.name = [] 
            hardware_cmd_msg.position = position_cmd_list
            hardware_cmd_msg.velocity = []
            hardware_cmd_msg.effort = []

            # 4. 发布指令
            self.command_pub.publish(hardware_cmd_msg)
            rospy.loginfo_throttle(1.0, f"  Step 3: Published command to {self.hardware_topic}")

        except Exception as e:
            # 捕获任何可能的错误并打印
            rospy.logerr(f"CRITICAL ERROR in target_callback: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = LinkerHandL10Controller()
        controller.run()
    except rospy.ROSInterruptException:
        pass