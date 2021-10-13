#!/usr/bin/env python2
import rospy
import math
from rospy.rostime import Time
import utils
import tf
import geometry_msgs.msg
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from matplotlib import pyplot as plt
from laser_line_extraction.msg import LineSegmentList


class CalDoorAngle:
    def __init__(self):
        self.point_sub = rospy.Subscriber(
            "/line_segments", LineSegmentList, self.pointCallback, queue_size=1)
        self.scan_sub = rospy.Subscriber(
            "/scan", LaserScan, self.scanCallback, queue_size=10)
        self.scan_pub = rospy.Publisher("/scan_v2",LaserScan,queue_size=10)
        self.angle_pub = rospy.Publisher("/door_angle",Float32,queue_size=10)
        self.tf_sub = tf.TransformListener()
        self.angle_min = 3

    def transformPoint(self, x, y):
        laser_point = geometry_msgs.msg.PointStamped()
        laser_point.header.frame_id = "base_footprint"
        laser_point.header.stamp = rospy.Time(0)
        laser_point.point.x = x
        laser_point.point.y = y
        laser_point.point.z = 0
        return self.tf_sub.transformPoint("/odom", laser_point)

    def scanCallback(self, data):
        data.angle_min = math.pi*270/180
        data.ranges = data.ranges[270:]
        data.intensities = data.intensities[270:]
        self.scan_pub.publish(data)

    def pointCallback(self, data):
        line_list = []
        for line in data.line_segments:
            odom_point = self.transformPoint( line.radius * math.cos(line.angle),  line.radius * math.sin(line.angle))
            if odom_point.point.y > 2.0 and odom_point.point.y < 3.5:
                # print(odom_point)
                line_list.append(line)

        res = []
        i = 0
        while i<len(line_list):
            j= i+1
            while j<len(line_list):
                angle = utils.GetCrossAngle(line_list[i],line_list[j])
                if angle > self.angle_min:
                    res.append(angle)
                j = j+1
            i = i+1
        if len(res)==1:
            self.angle_pub.publish(res[0])
        print(res)


if __name__ == "__main__":
    rospy.init_node('cal_door_angle', anonymous=True)
    cal = CalDoorAngle()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        r.sleep()
