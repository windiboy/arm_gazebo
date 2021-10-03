#!/usr/bin/env python
# coding=utf-8
import pyrealsense2 as rs
import rospy
import json
import numpy as np
import cv2
import tf2_ros
import tf
import math
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
            

if __name__ == "__main__":
    rospy.init_node('camera_base_transform')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    # 4.创建并组织被广播的消息
    tfs = TransformStamped()
    # --- 头信息
    tfs.header.frame_id = "base_link"
    tfs.header.stamp = rospy.Time.now()
    tfs.header.seq = 101
    # --- 子坐标系
    tfs.child_frame_id = rospy.get_param("child_frame", "camera_link")
    # --- 坐标系相对信息
    # ------ 偏移量
    tfs.transform.translation.x = -0.09
    tfs.transform.translation.y = 0.0
    tfs.transform.translation.z = 0.17
    # ------ 四元数
    qtn = tf.transformations.quaternion_from_euler(0,0,0)
    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]
    broadcaster.sendTransform(tfs)

    rospy.spin()
    

