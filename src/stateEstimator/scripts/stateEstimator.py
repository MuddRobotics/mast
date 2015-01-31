#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from mastMessages.msg import State, Control

current_state = State()
current_unc = State()

def talker():
    cur_state = rospy.Publisher('current_state', State, queue_size=10)
    cur_unc = rospy.Publisher('current_unc', State, queue_size=10)
    rospy.init_node('stateEstimator', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        cur_state.publish(current_state)
        cur_unc.publish(current_unc)
        rate.sleep()

def onNewControl(desired_flight):
    # Based on control input, update current state
    current_state.x += 1
    current_unc.x += 0.001

if __name__ == '__main__':
    try:
        rospy.Subscriber('control',  Control,  onNewState)
        talker()
    except rospy.ROSInterruptException:
        pass