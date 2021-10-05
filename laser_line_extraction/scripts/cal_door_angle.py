#!/usr/bin/env python2
import rospy
import math
from matplotlib import pyplot as plt
from laser_line_extraction.msg import LineSegmentList


class CalDoorAngle:
    def __init__(self):
        self.point_sub = rospy.Subscriber(
            "/line_segments", LineSegmentList, self.pointCallback)

    def pointCallback(self, data):
        for line in data.line_segments:
            x = []
            y = []
            x.append( line.radius * math.sin(line.angle))
            y.append( line.radius * math.cos(line.angle))
            x.append(line.start[0])
            x.append(line.start[1])
            y.append(line.end[0])
            y.append(line.end[1])
            plt.plot(x,y)
        plt.show()
        plt.close()


if __name__ == "__main__":
    rospy.init_node('cal_door_angle', anonymous=True)
    cal = CalDoorAngle()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        r.sleep()
