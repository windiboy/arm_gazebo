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

class CameraTransform:
    def __init__(self):
        rospy.init_node('camera_transform')
        self.image_pub = rospy.Publisher('/camera_transform/image_raw',Image,queue_size=1)
        self.point_sub = rospy.Subscriber("/object_detection/target_pose",Point,self.pointCallback)
        self.WIDTH = 1280
        self.HEIGHT = 720
        self.x = 0
        self.y = 0
        self.rate = rospy.Rate(30)

        # 相机配置
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.rgb8, 30)

        profile = self.pipeline.start(config)
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        # 获取相机内参
        intr = color_frame.profile.as_video_stream_profile().intrinsics
        camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                            'ppx': intr.ppx, 'ppy': intr.ppy,
                            'height': intr.height, 'width': intr.width,
                            'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                            }
        # 保存内参到本地
        with open('./intrinsics.json', 'w') as fp:
            json.dump(camera_parameters, fp)
        # 图像对齐
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        rospy.loginfo("[camera_transform] init successful!")
        self.run()

    def pointCallback(self,data):
        # 输入像素的x和y计算真实距离
        self.x = data.x
        self.y = data.y
    
    def tf_camera_static(self):
        broadcaster2 = tf2_ros.StaticTransformBroadcaster()
        # 4.创建并组织被广播的消息
        tfs2 = TransformStamped()
        # --- 头信息
        tfs2.header.frame_id = "base_link"
        tfs2.header.stamp = rospy.Time.now()
        tfs2.header.seq = 102
        # --- 子坐标系
        tfs2.child_frame_id = rospy.get_param("child_frame", "camera_link")
        # --- 坐标系相对信息
        # ------ 偏移量
        tfs2.transform.translation.x = -0.09
        tfs2.transform.translation.y = 0.0
        tfs2.transform.translation.z = 0.17
        # ------ 四元数
        qtn = tf.transformations.quaternion_from_euler(-math.pi/2,0,math.pi/2)
        tfs2.transform.rotation.x = qtn[0]
        tfs2.transform.rotation.y = qtn[1]
        tfs2.transform.rotation.z = qtn[2]
        tfs2.transform.rotation.w = qtn[3]

        # 5.广播器发送消息
        broadcaster2.sendTransform(tfs2)
        print tfs2

    def run(self):
        # 静态变换只需要发布一次
        self.tf_camera_static()
        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            aligned_depth_frame = aligned_frames.get_depth_frame()
            # 深度参数，像素坐标系转相机坐标系用到
            depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
            color_frame = aligned_frames.get_color_frame()

            # 深度图
            d = np.asanyarray(aligned_depth_frame.get_data())
            # 彩色图
            image_np = np.asanyarray(color_frame.get_data())
            # cv2.imshow("Color", image_np)
            # cv2.waitKey (0)

            image_raw = Image()
            image_raw.header.stamp = rospy.Time.now()
            image_raw.header.frame_id = 'color_optical_frame'
            image_raw.height = self.HEIGHT
            image_raw.width = self.WIDTH
            image_raw.encoding = "rgb8"
            image_raw.step = self.WIDTH*3
            temp = image_np.flatten()
            image_raw.data = temp.tolist()
            self.image_pub.publish(image_raw)

            if self.x != 0 and self.y !=0:
                # 计算相机坐标系下的位置
                dis = aligned_depth_frame.get_distance(int(self.x), int(self.y))
                camera_coordinate = rs.rs2_deproject_pixel_to_point(intrin=depth_intrin, pixel=[int(self.x), int(self.y)], depth=dis)
                print camera_coordinate
                # 如果门把手存在
                if 0 not in camera_coordinate:
                    # 发布门把手的坐标变换
                    broadcaster = tf2_ros.TransformBroadcaster()
                    tfs = TransformStamped()
                    # --- 头信息
                    tfs.header.frame_id = "camera_link"
                    tfs.header.stamp = rospy.Time.now()
                    tfs.header.seq = 101
                    # --- 子坐标系
                    tfs.child_frame_id = "door_hand"
                    # --- 坐标系相对信息
                    # ------ 偏移量
                    tfs.transform.translation.x = camera_coordinate[0]
                    tfs.transform.translation.y = camera_coordinate[1]
                    tfs.transform.translation.z = camera_coordinate[2]
                    # ------ 四元数
                    qtn = tf.transformations.quaternion_from_euler(0,0,0)
                    tfs.transform.rotation.x = qtn[0]
                    tfs.transform.rotation.y = qtn[1]
                    tfs.transform.rotation.z = qtn[2]
                    tfs.transform.rotation.w = qtn[3]
                    broadcaster.sendTransform(tfs)

            self.rate.sleep()
            

if __name__ == "__main__":
    camera = CameraTransform()
    

