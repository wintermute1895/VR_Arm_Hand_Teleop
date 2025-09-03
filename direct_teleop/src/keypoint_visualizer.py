#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from direct_teleop.msg import PicoHand
from geometry_msgs.msg import Point

class KeypointVisualizer:
    def __init__(self):
        rospy.init_node('keypoint_visualizer')
        
        self.marker_pub = rospy.Publisher('/debug/keypoint_markers', MarkerArray, queue_size=1)
        rospy.Subscriber('/pico/hand/keypoints', PicoHand, self.keypoints_callback)
        
        rospy.loginfo("Keypoint Visualizer is running. Waiting for data on /pico/hand/keypoints...")
        rospy.loginfo("Please run RViz and add a 'MarkerArray' display subscribed to /debug/keypoint_markers")

    def keypoints_callback(self, msg):
        marker_array = MarkerArray()
        
        # 遍历接收到的每一个点
        for i, point in enumerate(msg.keypoints):
            # --- 创建一个球体标记来代表关节点 ---
            sphere_marker = Marker()
            sphere_marker.header.frame_id = "pico_world" # 使用PICO的原始坐标系
            sphere_marker.header.stamp = rospy.Time.now()
            sphere_marker.ns = "keypoints"
            sphere_marker.id = i # 每个标记的ID必须唯一
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position = point
            sphere_marker.pose.orientation.w = 1.0
            sphere_marker.scale.x = 0.01 # 1cm的球
            sphere_marker.scale.y = 0.01
            sphere_marker.scale.z = 0.01
            sphere_marker.color.a = 0.8 # 透明度
            sphere_marker.color.r = 1.0
            sphere_marker.color.g = 0.0
            sphere_marker.color.b = 0.0 # 红色
            
            # --- 创建一个文本标记来显示索引号 ---
            text_marker = Marker()
            text_marker.header.frame_id = "pico_world"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "keypoint_indices"
            text_marker.id = i + 100 # 避免与球体的ID冲突
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = point.x
            text_marker.pose.position.y = point.y
            text_marker.pose.position.z = point.z + 0.02 # 让文字显示在球体上方
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.02 # 文字大小
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0 # 白色
            text_marker.text = str(i) # !! 核心：显示数组索引 !!

            marker_array.markers.append(sphere_marker)
            marker_array.markers.append(text_marker)
            
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        node = KeypointVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass