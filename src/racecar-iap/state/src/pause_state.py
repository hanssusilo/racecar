#!/usr/bin/env python

import roslib; roslib.load_manifest('state')
import rospy
import smach
import smach_ros

class Pause(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue1','continue2','continue3','continue4','continue5','continue1c','continue2c','continue3c','continue4c','paused','reset','failed'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state PAUSE')
        if self.counter < 3:
            self.counter += 1
            return 'finished'
        else:
            return 'failed'
