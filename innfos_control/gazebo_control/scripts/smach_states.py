#!/usr/bin/env python2
import math

import rospy
import smach
import copy
import geometry_msgs.msg
from gazebo_msgs.msg import ContactsState
from tf_conversions import transformations
from platform_control import PlatformControl
from arm_control import ArmControl
from gripper_control import GripperControl
from setting import HAND_POSE


class NAVIGATE_TO_OUTLET(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], output_keys=['NAVIGATE_TO_OUTLET_output'])
        self.counter = 0
        self.name = "NAVIGATE_TO_OUTLET"
        self.nav = PlatformControl()
        self.arm = ArmControl()
        self.nav_goal = [HAND_POSE[0], HAND_POSE[1]-0.3, 0.0, math.pi/2]

    def execute(self, userdata):
        rospy.loginfo('Executing state {}'.format(self.name))
        rospy.loginfo('Naviting to {}'.format(self.nav_goal))
        if self.nav.goto(self.nav_goal):
            return 'succeeded'
        else:
            return 'failed'


class FETCH_DOOR_HAND(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.counter = 0
        self.name = "FETCH_DOOR_HAND"
        self.arm = ArmControl()

    def execute(self, userdata):
        rospy.loginfo('Executing state {}'.format(self.name))

        pose_goal = geometry_msgs.msg.Pose()
        q = transformations.quaternion_from_euler(HAND_POSE[3], HAND_POSE[4], HAND_POSE[5])
        pose_goal.position.x = HAND_POSE[0]
        pose_goal.position.y = HAND_POSE[1]
        pose_goal.position.z = HAND_POSE[2]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]

        waypoints = []
        point_1 = copy.deepcopy(pose_goal)
        point_1.position.y -= 0.1
        waypoints.append(point_1)
        waypoints.append(pose_goal)

        (plan, fraction) = self.arm.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.02,        # eef_step
            0.0)         # jump_threshold
        rospy.loginfo("[{}] waypoints: {}".format(self.name, waypoints))
        rospy.loginfo("[{}] Cartesian path {:.2f}% acheived".format(self.name, fraction*100))
        if fraction > 0.5:
            self.arm.move_group.execute(plan, wait=True)
            return 'succeeded'
        else:
            return 'failed'


class GRIPPER_CLOSE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.counter = 0
        self.name = "GRIPPER_CLOSE"
        self.gripper = GripperControl()
        self.catched = False
        self.contact_sub = rospy.Subscriber("/bumper",ContactsState,self.pointCallback)
    
    def pointCallback(self, data):
        if len(data.states) >0:
            self.catched = True
        else:
            self.catched = False

    def execute(self, userdata):
        rospy.loginfo('Executing state {}'.format(self.name))
        goal = 0.5
        while goal<=0.8 and self.catched == False:
            self.gripper.move_group.go([goal], wait=True)
            goal += 0.05
        if self.catched:
            return 'succeeded'
        else:
            return 'failed'

class GRIPPER_OPEN(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.name = "GRIPPER_OPEN"
        self.gripper = GripperControl()

    def execute(self, userdata):
        rospy.loginfo('Executing state {}'.format(self.name))
        self.gripper.move_group.go([0.0], wait=True)
        return 'succeeded'


class NAVIGATE_CIRCLE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.name = "NAVIGATE_CIRCLE"
        self.nav = PlatformControl()

    def execute(self, userdata):
        rospy.loginfo('Executing state {}'.format(self.name))
        if self.nav.pull_door_circle_v2():
            return 'succeeded'
        else:
            return 'failed'