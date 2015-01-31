#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from mastMessages.msg import State, Control, Flight

control_msg = Control()

def talker():
    pub = rospy.Publisher('control', Control, queue_size=10)
    rospy.init_node('controls', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(control_msg)
        pub.publish(control_msg)
        rate.sleep()

def onNewFlight(desired_flight):
    # Based on desired flight, do something to control msg
    control_msg.delta_r += 1

def onNewState(current_state):
    # Based on Current State, do something to control msg
    control_msg.delta_t += 1

if __name__ == '__main__':
    try:
        rospy.Subscriber('desired_flight', Flight, onNewFlight)
        rospy.Subscriber('current_state',  State,  onNewState)
        talker()
    except rospy.ROSInterruptException:
        pass