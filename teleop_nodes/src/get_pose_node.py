#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose

def get_current_pose():
    # 初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('get_pose_node', anonymous=True)

    # 创建一个arm组的接口
    arm_group = moveit_commander.MoveGroupCommander("arm")

    # 获取并打印当前末端的位姿
    rospy.loginfo("="*50)
    rospy.loginfo("Reading current end-effector pose for 'arm' group...")
    
    current_pose = arm_group.get_current_pose().pose
    
    rospy.loginfo("Position (x, y, z):")
    rospy.loginfo(f"  x: {current_pose.position.x}")
    rospy.loginfo(f"  y: {current_pose.position.y}")
    rospy.loginfo(f"  z: {current_pose.position.z}")
    
    rospy.loginfo("Orientation (x, y, z, w) - Quaternion:")
    rospy.loginfo(f"  x: {current_pose.orientation.x}")
    rospy.loginfo(f"  y: {current_pose.orientation.y}")
    rospy.loginfo(f"  z: {current_pose.orientation.z}")
    rospy.loginfo(f"  w: {current_pose.orientation.w}")
    
    rospy.loginfo("="*50)
    rospy.loginfo("You can now copy these values into your pico_data_simulator_node.py")
    rospy.loginfo("Node will shut down in 5 seconds.")
    rospy.sleep(5) # 暂停5秒，让你有时间看清楚

    # 清理
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        get_current_pose()
    except rospy.ROSInterruptException:
        pass