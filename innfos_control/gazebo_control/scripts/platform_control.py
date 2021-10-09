#!/usr/bin/env python2
import math
import rospy
import tf
import actionlib
import time
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from tf_conversions import transformations
from math import pi, sin
from std_msgs.msg import String
from setting import DOOR_JOINT_POSE


class PlatformControl:
    def __init__(self):
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.tf_sub = tf.TransformListener()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s" %
                      (status, result))

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        rospy.loginfo("[Navi] navigation feedback\r\n%s" % feedback)

    def pull_door_circle(self):
        try:
            (trans, rot) = self.tf_sub.lookupTransform(
                "/odom", "/base_footprint", rospy.Time(0))
            x = trans[0]-DOOR_JOINT_POSE[0]
            y = trans[1]-DOOR_JOINT_POSE[1]
            radius = math.sqrt(x*x + y*y)
            angle = math.atan(y/x)
            while angle < math.pi/2:
                angle += 0.01
                radius *= 0.9
                goal = [-radius * math.cos(angle)+DOOR_JOINT_POSE[0],
                        -radius * math.sin(angle)+DOOR_JOINT_POSE[1], 0.0, angle+math.pi/2]
                rospy.loginfo("[Navi] pull_door_circle goto:{}".format(goal))
                self.goto(goal)
        except Exception as e:
            rospy.logerr("[Navi] pull_door_circle err {}".format(e))

    def pull_door_circle_v2(self):
        vel = Twist()
        vel.linear.x = -0.05
        vel.angular.z = 0.2
        self.cmd_pub.publish(vel)
        time.sleep(10)
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


if __name__ == "__main__":
    rospy.init_node('PlatformControl', anonymous=True)
    navi = PlatformControl()
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        navi.pull_door_circle_v2()
        r.sleep()
