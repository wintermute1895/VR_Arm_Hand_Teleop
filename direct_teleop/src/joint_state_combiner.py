#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

class JointStateCombiner:
    def __init__(self):
        rospy.init_node('joint_state_combiner', anonymous=True)
        self.arm_state = None
        self.hand_state = None
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        rospy.Subscriber('/arm_control/target_joint_states', JointState, self.arm_callback)
        rospy.Subscriber('/hand_retargeting/target_joint_states', JointState, self.hand_callback)
        rospy.Timer(rospy.Duration(1.0/50.0), self.publish_loop) # 以50Hz频率尝试发布
        rospy.loginfo("Joint State Combiner is running and waiting for data...")

    def arm_callback(self, msg):
        self.arm_state = msg

    def hand_callback(self, msg):
        self.hand_state = msg

    def publish_loop(self, event):
        if self.arm_state is not None and self.hand_state is not None:
            combined_msg = JointState()
            combined_msg.header.stamp = rospy.Time.now()
            combined_msg.name = self.arm_state.name + self.hand_state.name
            combined_msg.position = list(self.arm_state.position) + list(self.hand_state.position)
            self.pub.publish(combined_msg)

if __name__ == '__main__':
    try:
        combiner = JointStateCombiner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass