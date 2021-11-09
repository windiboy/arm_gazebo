#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import roslib
import rospy
import math
import tf
import csv
import time
import geometry_msgs.msg

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('my_tf_listener')

    # 创建tf的监听器
    listener = tf.TransformListener()

    first_tf = 'base_link'
    second_tf = 'gripper'
    date_time_str = time.strftime("%m_%d_%H_%M_%S", time.localtime(time.time()))
    file_name = '/home/yang/yang_ws/src/arm_gazebo/innfos_control/gazebo_control/data/'+'arm_'+date_time_str+".csv"

    headers = ['time',second_tf+'_position_x',second_tf+'_position_y',second_tf+'_position_z',
                second_tf+'_rot_x',second_tf+'_rot_y',second_tf+'_rot_z',second_tf+'_rot_w']

    with open(file_name,'w') as f:
        f_csv = csv.writer(f)
        f_csv.writerow(headers)

    t = time.time()
    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        # try:
        #     (trans,rot) = listener.lookupTransform('/map', '/base_link', t)
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     continue

        try:
            (trans2,rot2) = listener.lookupTransform('/base_link', '/robotiq_85_base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        row = [time.time()-t] + trans2 + rot2
        print(row)
        with open(file_name,'a+') as f:
            f_csv = csv.writer(f)
            f_csv.writerow(row)

        rate.sleep()