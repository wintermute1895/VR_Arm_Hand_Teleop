#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trac_ik_python.trac_ik import IK

class ArmRvizController:
    def __init__(self):
        rospy.init_node('arm_rviz_controller')

        self.base_link = "base_link" 
        self.end_effector_link = "Link7" 
        self.urdf_string = rospy.get_param("/robot_description")

        self.ik_solver = IK(self.base_link, self.end_effector_link, urdf_string=self.urdf_string)
        
        self.target_arm_angles = [0.0] * self.ik_solver.number_of_joints
        self.arm_joint_names = self.ik_solver.joint_names

        self.arm_joint_pub = rospy.Publisher('/arm_control/target_joint_states', JointState, queue_size=10)
        rospy.Subscriber('/pico/hand/pose', PoseStamped, self.pico_pose_callback, queue_size=1)
        
        self.timer = rospy.Timer(rospy.Duration(1.0/50.0), self.publish_arm_states)
        rospy.loginfo("Arm RViz Controller is ready.")

    def pico_pose_callback(self, msg):
        target_joints = self.ik_solver.get_ik(self.target_arm_angles,
                                              msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                                              msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        if target_joints:
            self.target_arm_angles = target_joints

    def publish_arm_states(self, event):
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        js_msg.name = self.arm_joint_names
        js_msg.position = self.target_arm_angles
        self.arm_joint_pub.publish(js_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = ArmRvizController()
        controller.run()
    except rospy.ROSInterruptException:
        pass