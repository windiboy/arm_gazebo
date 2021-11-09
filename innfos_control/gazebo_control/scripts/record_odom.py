#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import roslib
import rospy
import math
import tf
import csv
import time
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class RecordOdom:
    def __init__(self):
        self.odom_sub = rospy.Subscriber(
            "/odom", Odometry, self.OdomCallback, queue_size=10)
        self.angle_sub = rospy.Subscriber(
            "/door_angle", Float32, self.AngleCallback, queue_size=10)
        second_tf = 'odom'
        date_time_str = time.strftime("%m_%d_%H_%M_%S", time.localtime(time.time()))
        self.file_name = '/home/yang/yang_ws/src/arm_gazebo/innfos_control/gazebo_control/data/'+'odom_'+date_time_str+".csv"

        headers = ['time',second_tf+'_position_x',second_tf+'_position_y',second_tf+'_position_z',
                    second_tf+'_rot_x',second_tf+'_rot_y',second_tf+'_rot_z',second_tf+'_rot_w']

        with open(self.file_name,'w') as f:
            f_csv = csv.writer(f)
            f_csv.writerow(headers)

        self.angle_file_name = '/home/yang/yang_ws/src/arm_gazebo/innfos_control/gazebo_control/data/'+'angle_'+date_time_str+".csv"
        self.t = time.time()
        self.rate = rospy.Rate(30.0)

    def OdomCallback(self,data):
        row = [time.time()-self.t] + [data.pose.pose.position.x] + [data.pose.pose.position.y] + [data.pose.pose.position.z] + [data.pose.pose.orientation.x] + [data.pose.pose.orientation.y] + [data.pose.pose.orientation.z] + [data.pose.pose.orientation.w]
        print(row)
        with open(self.file_name,'a+') as f:
            f_csv = csv.writer(f)
            f_csv.writerow(row)
    
    def AngleCallback(self,data):
        row = [time.time()-self.t] + [data.data]
        print(row)
        with open(self.angle_file_name,'a+') as f:
            f_csv = csv.writer(f)
            f_csv.writerow(row)
    

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('my_tf_listener')
    my = RecordOdom()
    
    while not rospy.is_shutdown():
        my.rate.sleep()