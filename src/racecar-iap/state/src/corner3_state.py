#!/usr/bin/env python

import roslib; roslib.load_manifest('state')
import rospy
import smach
import smach_ros

class Corner3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished','paused','reset','failed'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state CORNER3')
        if self.counter < 3:
            self.counter += 1
            return 'finished'
        else:
            return 'failed'
