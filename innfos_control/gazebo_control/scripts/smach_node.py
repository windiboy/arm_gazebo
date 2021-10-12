#!/usr/bin/env python2
import math
import rospy
import smach
import smach_ros
import geometry_msgs.msg
import smach_states
from tf_conversions import transformations


class Smach_Node():
    def __init__(self):
        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('NAVIGATE_TO_OUTLET', smach_states.NAVIGATE_TO_OUTLET(),
                                   transitions={'succeeded': 'FETCH_DOOR_HAND', 'failed': 'failed'})
            smach.StateMachine.add('FETCH_DOOR_HAND', smach_states.FETCH_DOOR_HAND(),
                                   transitions={'succeeded': 'GRIPPER_CLOSE', 'failed': 'failed'})
            smach.StateMachine.add('GRIPPER_CLOSE', smach_states.GRIPPER_CLOSE(),
                                   transitions={'succeeded': 'NAVIGATE_CIRCLE', 'failed': 'failed'})
            smach.StateMachine.add('NAVIGATE_CIRCLE', smach_states.NAVIGATE_CIRCLE(),
                                   transitions={'succeeded': 'GRIPPER_OPEN', 'failed': 'failed'})
            smach.StateMachine.add('GRIPPER_OPEN', smach_states.GRIPPER_OPEN(),
                                   transitions={'succeeded': 'PREPARE_FOR_NEXT'})
            smach.StateMachine.add('PREPARE_FOR_NEXT', smach_states.PREPARE_FOR_NEXT(),
                                   transitions={'succeeded': 'NAVIGATE_TO_DOOR', 'failed': 'failed'})
            smach.StateMachine.add('NAVIGATE_TO_DOOR', smach_states.NAVIGATE_TO_DOOR(),
                                   transitions={'succeeded': 'NAVIGATE_TO_OTHERROOM', 'failed': 'failed'})
            smach.StateMachine.add('NAVIGATE_TO_OTHERROOM', smach_states.NAVIGATE_TO_OTHERROOM(),
                                   transitions={'succeeded': 'succeeded', 'failed': 'failed'})

        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer(
            'my_smach_introspection_server', sm, '/SM_ROOT')
        sis.start()

        # Execute SMACH plan
        outcome = sm.execute()

        # Wait for ctrl-c to stop the application
        rospy.spin()
        sis.stop()


if __name__ == "__main__":
    rospy.init_node('smach_node', anonymous=True)
    smach_node = Smach_Node()
