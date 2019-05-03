#!/usr/bin/env python

import roslib
import rospy

from kobuki_msgs.msg import BumperEvent

#class that handles the bumper sensor
class BumperAction:

    _bumper_collision = False #collision variable

    def __init__(self):
        self.bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.bumper_collision)
        self._bumper_collision = False

    def bumper_collision(self, data): #listener of the sesnor data

        if data.bumper == 0 or data.bumper == 1 or data.bumper == 2 or data.state == BumperEvent.PRESSED or data.state == BumperEvent.RELEASED: #verify if the sensor switch is pressed
            self._bumper_collision = True
        else: #sensor not pressed
            self._bumper_collision = False
        #rospy.loginfo("%d"%data.bumper)

    def get_bumper_collision(self): #return the value of collision 
        return self._bumper_collision

    def reset_bumper_collision(self):
        self._bumper_collision = False

if __name__ == '__main__':
    rospy.init_node('bumper_action_subscriber', anonymous = True)
    bumper_Action = BumperAction()
    rospy.spin()
