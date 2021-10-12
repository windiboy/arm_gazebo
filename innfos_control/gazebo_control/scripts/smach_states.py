#!/usr/bin/env python2
import math
import threading
import time
import rospy
import smach
import copy
import geometry_msgs.msg
from gazebo_msgs.msg import ContactsState
from tf_conversions import transformations
from platform_control import PlatformControl, NavThread
from arm_control import ArmControl, ArmThread
from gripper_control import GripperControl
from setting import HAND_POSE
from utils import ListToPose


class NAVIGATE_TO_OUTLET(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], output_keys=[
                             'NAVIGATE_TO_OUTLET_output'])
        self.counter = 0
        self.name = "NAVIGATE_TO_OUTLET"
        self.nav = PlatformControl()
        self.arm = ArmControl()
        self.nav_goal = [HAND_POSE[0]+0.2, HAND_POSE[1]-0.3, 0.0, math.pi*2/3]

    def execute(self, userdata):
        rospy.loginfo('Executing state {}'.format(self.name))
        rospy.loginfo('Naviting to {}'.format(self.nav_goal))
        if self.nav.goto(self.nav_goal):
            return 'succeeded'
        else:
            return 'failed'


class NAVIGATE_TO_DOOR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.counter = 0
        self.name = "NAVIGATE_TO_DOOR"
        self.nav = PlatformControl()
        self.arm = ArmControl()
        self.nav_goal = [HAND_POSE[0]-0.2, HAND_POSE[1]-0.3, 0.0, math.pi/2]

    def execute(self, userdata):
        rospy.loginfo('Executing state {}'.format(self.name))
        rospy.loginfo('Naviting to {}'.format(self.nav_goal))
        if self.nav.goto(self.nav_goal):
            return 'succeeded'
        else:
            return 'failed'


class NAVIGATE_TO_OTHERROOM(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.counter = 0
        self.name = "NAVIGATE_TO_OTHERROOM"
        self.nav = PlatformControl()
        self.arm = ArmControl()
        self.nav_goal = [0, 5, 0.0, math.pi/2]

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

        pose_goal = ListToPose(HAND_POSE)

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
        rospy.loginfo("[{}] Cartesian path {:.2f}% acheived".format(
            self.name, fraction*100))
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
        self.contact_sub = rospy.Subscriber(
            "/bumper", ContactsState, self.pointCallback)

    def pointCallback(self, data):
        self.catched = False
        if len(data.states) > 10:
            if data.states[-1].depths[0] > 0.02:
                # rospy.loginfo("[{}] depth: {}".format(
                #     self.name, data.states[-1].depths))
                self.catched = True

    def execute(self, userdata):
        rospy.loginfo('Executing state {}'.format(self.name))
        goal = 0.65
        self.gripper.move_group.go([goal], wait=True)
        # while goal <= 0.8 and self.catched == False:
        #     self.gripper.move_group.go([goal], wait=True)
        #     goal += 0.01
        return 'succeeded'
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
        self.arm = ArmControl()

    def execute(self, userdata):
        rospy.loginfo('Executing state {}'.format(self.name))

        try:
            (trans, rot) = self.nav.tf_sub.lookupTransform(
                "/base_link", "/ee_link", rospy.Time(0))
        except Exception as e:
            rospy.logerr("[{}] tf err: {}".format(self.name, e))
            return False
        trans[0] += 0.05
        trans[1] -= 0.05
        point_2 = ListToPose(trans+HAND_POSE[3:])
        rospy.loginfo("[{}] goal: {}".format(self.name, point_2))
        self.arm.move_group.set_pose_target(point_2)
        if True:
            threads = []

            thread1 = ArmThread(self.arm.move_group)
            thread2 = NavThread()

            # thread1.start()
            thread2.start()

            # threads.append(thread1)
            threads.append(thread2)

            for t in threads:
                t.join()
            return 'succeeded'
        else:
            return 'failed'


class PREPARE_FOR_NEXT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.name = "PREPARE_FOR_NEXT"
        self.nav = PlatformControl()
        self.arm = ArmControl()

    def execute(self, userdata):
        rospy.loginfo('Executing state {}'.format(self.name))

        jobs = []
        jobs.append(threading.Thread(target=self.nav.move_back, args=(5.0,)))
        jobs[0].start()
        jobs.append(threading.Thread(target=self.arm.move_group.go(),
                    args=([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],)))
        time.sleep(2)
        jobs[1].start()

        for j in jobs:
            j.join()
        return "succeeded"
