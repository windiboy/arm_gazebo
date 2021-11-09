#!/usr/bin/env python2
import math
import rospy
import tf
import actionlib
import time
import threading
from actionlib_msgs.msg import *
from gazebo_msgs.msg import ContactsState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from tf_conversions import transformations
from math import pi, sin
from std_msgs.msg import String,Float32


class PlatformControl:
    def __init__(self):
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.tf_sub = tf.TransformListener()
        self.tf_sub.waitForTransform("/odom", "/base_footprint", rospy.Time(), rospy.Duration(4.0))
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.contact_sub = rospy.Subscriber(
            "/bumper", ContactsState, self.pointCallback)
        self.catched = True
        self.start_time_cycle = rospy.Time.now()
        self.angle_sub = rospy.Subscriber(
            "/door_angle", Float32, self.AngleCallback, queue_size=10)
        self.angle = 0

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s" %
                      (status, result))

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        pass
        # rospy.loginfo("[Navi] navigation feedback\r\n%s" % feedback)
    
    def pointCallback(self, data):
        if len(data.states) == 0:
            self.catched = False

    def AngleCallback(self,data):
        self.angle = data.data

    def move_back(self, delay=5.0):
        vel = Twist()
        vel.linear.x = -0.2
        self.cmd_pub.publish(vel)
        time.sleep(delay)
        vel.linear.x = 0.0
        self.cmd_pub.publish(vel)
        return True

    def pull_door_circle_v2(self):
        vel = Twist()
        used_time = rospy.Time.now()-self.start_time_cycle
        while self.catched and (used_time < rospy.Duration(25)):
            used_time = rospy.Time.now()-self.start_time_cycle
            rospy.loginfo("[pull_door_circle_v2] used_time {}, max time {}".format(used_time,rospy.Duration(30)))
            vel.linear.x = -0.05
            vel.angular.z = 0.01
            self.cmd_pub.publish(vel)
            if self.angle >60:
                break
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.cmd_pub.publish(vel)
        return True

    def goto(self, p):
        rospy.loginfo("[Navi] goto %s" % p)

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        goal.target_pose.pose.position.z = p[2]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[3])
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(
            goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return False
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!" % p)
                return True
        return False

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True


class NavThread (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.nav = PlatformControl()

    def run(self):
        self.nav.pull_door_circle_v2()
        rospy.loginfo("[NavThread] exit")


if __name__ == "__main__":
    rospy.init_node('PlatformControl', anonymous=True)
    navi = PlatformControl()
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        navi.pull_door_circle_v2()
        r.sleep()
