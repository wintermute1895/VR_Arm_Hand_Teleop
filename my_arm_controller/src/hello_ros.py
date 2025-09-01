#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

if __name__ == '__main__':
    try:
        # 唯一的动作：初始化节点
        rospy.init_node('hello_ros_node', anonymous=True)
        
        # 如果初始化成功，就打印这句话
        rospy.loginfo("Hello, ROS! My node is alive.")
        
        # 让节点保持运行，直到被Ctrl+C关闭
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
