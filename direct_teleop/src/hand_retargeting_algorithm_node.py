#!/usr/bin/env python3
import rospy
import time
import numpy as np
from sensor_msgs.msg import JointState
from direct_teleop.msg import PicoHand

# 确保导入路径正确
from hand_retargeting_lib.L10 import config_l10 as config
from hand_retargeting_lib.L10.robothandmodel_l10 import RobotHandModel_L10
from hand_retargeting_lib.retargeting import HandOptimizer

class HandRetargetingAlgorithmNode:
    def __init__(self):
        rospy.init_node('hand_retargeting_algorithm_node')

        try:
            # self.config = config
            self.robot_model = RobotHandModel_L10(config)
            self.optimizer = HandOptimizer(self.robot_model, config)
            self.is_calibrated = False

            self.hand_joint_pub = rospy.Publisher('/hand_retargeting/target_joint_states', JointState, queue_size=10)
            rospy.Subscriber('/pico/hand/keypoints', PicoHand, self.pico_hand_callback, queue_size=1)
            
            rospy.loginfo("Hand Retargeting Algorithm Node is ready.")
        except Exception as e:
            rospy.logerr(f"Error during initialization of HandRetargetingAlgorithmNode: {e}")

    def pico_hand_callback(self, msg):
        try: # !! 新增：用try...except包裹整个回调函数，防止它静默失败 !!
            if len(msg.keypoints) == 0:
                rospy.logwarn_throttle(2.0, "Received PicoHand message with 0 keypoints. Skipping.")
                return

            w_lhs = np.array([[p.x, p.y, p.z] for p in msg.keypoints])

            if not self.is_calibrated:
                rospy.loginfo_once("First frame received, attempting to calibrate hand model...")
                self.robot_model.calibrate(w_lhs)
                self.is_calibrated = True
                rospy.loginfo("Hand model calibrated successfully!")
                return
            if self.is_calibrated:
                # !! 新增：记录优化开始时间 !!
                start_time = time.time()

                final_q_dict, loss = self.optimizer.optimize_q(w_lhs)

                # !! 新增：计算并打印耗时 !!
                duration_ms = (time.time() - start_time) * 1000
                rospy.loginfo_throttle(1.0, f"Optimization took: {duration_ms:.2f} ms.")

            if loss != -1.0:
                js_msg = JointState()
                js_msg.header.stamp = rospy.Time.now()
                
                positions = []
                names = []
                for finger in config.FINGER_NAMES:
                    finger_names = config.JOINT_INFO[finger]['names']
                    finger_angles = final_q_dict.get(finger, []) # 使用.get()更安全
                    if len(finger_names) == len(finger_angles):
                        names.extend(finger_names)
                        positions.extend(finger_angles)
                    
                js_msg.name = names
                js_msg.position = positions
                
                self.hand_joint_pub.publish(js_msg)
            else:
                rospy.logwarn_throttle(1.0, "Hand optimization failed.")

        except Exception as e:
            rospy.logerr(f"Error in pico_hand_callback: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = HandRetargetingAlgorithmNode()
        node.run()
    except rospy.ROSInterruptException:
        pass