#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from mastMessages.msg import State, Flight

flight_msg = Flight()

def talker():
    pub = rospy.Publisher('desired_flight', Flight, queue_size=10)
    rospy.init_node('point_tracker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(flight_msg)
        rate.sleep()

def onNewDesState(desired_state):
    # Based on desired state, do something to flight msg
    flight_msg.turning_radius += 1

def onNewCurrState(current_state):
    # Based on current state, do something to flight msg
    flight_msg.airspeed += 1

if __name__ == '__main__':
    try:
        rospy.Subscriber('desired_state', State, onNewDesState)
        rospy.Subscriber('current_state',  State,  onNewCurrState)
        talker()
    except rospy.ROSInterruptException:
        pass