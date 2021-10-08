#!/usr/bin/env python2
import rospy
import smach
import smach_ros
import geometry_msgs.msg
from platform_control import PlatformControl
from arm_control import ArmControl
from gripper_control import GripperControl

class NAVIGATE_TO_OUTLET(smach.State):
          def __init__(self):
                    smach.State.__init__(self, outcomes=['succeeded','failed'])
                    self.counter = 0
                    self.name = "NAVIGATE_TO_OUTLET"
                    self.nav = PlatformControl()
                    self.arm = ArmControl()
                    self.nav_goal = [-0.32, 2.61, 0.0, 1.57]
                    
          def execute(self, userdata):
                    rospy.loginfo('Executing state {}'.format(self.name))
                    rospy.loginfo('Naviting to {}'.format(self.nav_goal))
                    if self.nav.goto(self.nav_goal):
                              return 'succeeded'
                    else:
                              return 'failed'   

class FETCH_DOOR_HAND(smach.State):
          def __init__(self):
                    smach.State.__init__(self, outcomes=['succeeded','failed'])
                    self.counter = 0
                    self.name = "FETCH_DOOR_HAND"
                    self.arm = ArmControl()
                    
          def execute(self, userdata):
                    rospy.loginfo('Executing state {}'.format(self.name))
                    if self.nav.goto([-0.32, 2.61, 0.0, 1.57]):
                              return 'succeeded'
                    else:
                              return 'failed'   