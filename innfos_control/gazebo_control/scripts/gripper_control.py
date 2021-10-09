#!/usr/bin/env python2
import rospy
import time
import moveit_commander


class GripperControl:
    def __init__(self):
        self.move_group = moveit_commander.MoveGroupCommander("gripper")
        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.go([0.0], wait=True)
        self.move_group.stop()
