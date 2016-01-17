#!/usr/bin/env python

import roslib; roslib.load_manifest('state')
import rospy
import smach
import smach_ros

from zone1_state import Zone1
from zone2_state import Zone2
from zone3_state import Zone3
from zone4_state import Zone4
from zone5_state import Zone5

from corner1_state import Corner1
from corner2_state import Corner2
from corner3_state import Corner3
from corner4_state import Corner4

from pause_state import Pause


class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished','paused','reset','failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state START')

	#Initialize variables and stuff

        return 'finished'


# main
def main():
    rospy.init_node('start')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('START', Start(), 
                               transitions={'finished':'ZONE1',
                                            'paused':'PAUSE',
                                            'reset':'START',
                                            'failed':'failed'})
        smach.StateMachine.add('ZONE1', Zone1(), 
                               transitions={'finished':'CORNER1',
                                            'paused':'PAUSE',
                                            'reset':'START',
                                            'failed':'failed'})
        smach.StateMachine.add('CORNER1', Corner1(), 
                               transitions={'finished':'ZONE2',
                                            'paused':'PAUSE',
                                            'reset':'START',
                                            'failed':'failed'})
        smach.StateMachine.add('ZONE2', Zone2(), 
                               transitions={'finished':'CORNER2',
                                            'paused':'PAUSE',
                                            'reset':'START',
                                            'failed':'failed'})
        smach.StateMachine.add('CORNER2', Corner2(), 
                               transitions={'finished':'ZONE3',
                                            'paused':'PAUSE',
                                            'reset':'START',
                                            'failed':'failed'})
        smach.StateMachine.add('ZONE3', Zone3(), 
                               transitions={'finished':'CORNER3',
                                            'paused':'PAUSE',
                                            'reset':'START',
                                            'failed':'failed'})
        smach.StateMachine.add('CORNER3', Corner3(), 
                               transitions={'finished':'ZONE4',
                                            'paused':'PAUSE',
                                            'reset':'START',
                                            'failed':'failed'})
        smach.StateMachine.add('ZONE4', Zone4(), 
                               transitions={'finished':'CORNER4',
                                            'paused':'PAUSE',
                                            'reset':'START',
                                            'failed':'failed'})
        smach.StateMachine.add('CORNER4', Corner4(), 
                               transitions={'finished':'ZONE5',
                                            'paused':'PAUSE',
                                            'reset':'START',
                                            'failed':'failed'})
	smach.StateMachine.add('ZONE5', Zone5(), 
                               transitions={'finished':'finished',
                                            'paused':'PAUSE',
                                            'reset':'START',
                                            'failed':'failed'})
	smach.StateMachine.add('PAUSE', Pause(), 
                               transitions={'continue1':'ZONE1',
                                            'continue2':'ZONE2',
                                            'continue3':'ZONE3',
                                            'continue4':'ZONE4',
                                            'continue5':'ZONE5',
                                            'continue1c':'CORNER1',
                                            'continue2c':'CORNER2',
                                            'continue3c':'CORNER3',
                                            'continue4c':'CORNER4',
                                            'paused':'PAUSE',
                                            'reset':'START',
                                            'failed':'failed'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
