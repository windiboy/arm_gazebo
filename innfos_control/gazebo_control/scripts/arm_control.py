#!/usr/bin/env python2
import rospy
import time
import tf
import threading
import math
import copy
import moveit_commander
import geometry_msgs
from utils import ListToPose
from setting import HAND_POSE
from moveit_msgs.msg import OrientationConstraint, Constraints


class ArmControl:
    def __init__(self, init=True):
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.tf_sub = tf.TransformListener()
        self.tf_sub.waitForTransform(
            "/odom", "/ee_link", rospy.Time(), rospy.Duration(4.0))
        self.move_group.set_max_velocity_scaling_factor(1)
        if init:
            self.move_group.go([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], wait=True)
            self.move_group.stop()

    def test(self):
        try:
            (trans, rot) = self.tf_sub.lookupTransform(
                "/odom", "/ee_link", rospy.Time(0))
        except Exception as e:
            rospy.logerr("[{}] tf err: {}".format("arm_test", e))
            return False
        trans[0] += 0.05
        trans[1] -= 0.1
        point_2 = ListToPose(trans+HAND_POSE[3:])
        rospy.loginfo("[{}] goal: {}".format("arm_test", point_2))
        self.move_group.set_pose_target(point_2)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()


class ArmThread (threading.Thread):
    def __init__(self, move_group):
        threading.Thread.__init__(self)
        self.move_group = move_group

    def move_circle(self):
        waypoints = []
        target = ListToPose(HAND_POSE)
        r = 0.1
        center_x = HAND_POSE[0]+r
        center_y = HAND_POSE[1]

        ocm = OrientationConstraint()
        ocm.link_name = "ee_link"
        ocm.orientation = target.orientation
        ocm.absolute_x_axis_tolerance = 0.1
        ocm.absolute_y_axis_tolerance = 0.1
        ocm.absolute_z_axis_tolerance = 0.0
        ocm.weight = 1.0
        constraints = Constraints()
        constraints.orientation_constraints = [ocm]
        self.move_group.set_path_constraints(constraints)

        for i in [x * 0.01 for x in range(350, 471)]:
            target.position.x = center_x + r*math.cos(i)
            target.position.y = center_y + r*math.sin(i)
            waypoints.append(copy.deepcopy(target))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.02,        # eef_step
            0.0)         # jump_threshold
        rospy.loginfo("[{}] Cartesian path {:.2f}% acheived".format(
            "move_circle", fraction*100))
        if fraction > 0.5:
            self.move_group.execute(plan, wait=True)
        self.move_group.clear_path_constraints()

    def run(self):
        self.move_group.set_max_velocity_scaling_factor(0.01)
        self.move_group.set_max_acceleration_scaling_factor(0.01)
        self.move_circle()
        # self.move_group.go(wait=True)
        # self.move_group.stop()
        # self.move_group.clear_pose_targets()
        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.set_max_acceleration_scaling_factor(1)
        rospy.loginfo("[ArmThread] exit")


if __name__ == "__main__":
    rospy.init_node('ArmControl', anonymous=True)
    arm = ArmControl(init=False)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        # arm.move_circle()
        break
        r.sleep()
