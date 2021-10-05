#!/usr/bin/env python2
import rospy
import time
import moveit_commander

rospy.init_node('gripper_go')
grp = moveit_commander.MoveGroupCommander("gripper")
grp.set_max_velocity_scaling_factor(1)
grp.set_named_target('close')
grp.go(wait=True)
grp.set_joint_value_target('robotiq_85_left_knuckle_joint', 0.0)
grp.go(wait=True)
