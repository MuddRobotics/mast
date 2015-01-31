#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from mastMessages.msg import State, Position

des_state = State()

def talker():
    pub = rospy.Publisher('desired_state', State, queue_size=10)
    rospy.init_node('planner', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(des_state)
        rate.sleep()

def onNewPOI(poi):
    # Based on the POI, do something to flight msg
    des_state.y += 1

def onNewCurrState(current_state):
    # Based on current state, do something to flight msg
    des_state.x = current_state.x + 1

if __name__ == '__main__':
    try:
        rospy.Subscriber('interest_pos', Position, onNewPOI)
        rospy.Subscriber('current_state', State, onNewCurrState)
        talker()
    except rospy.ROSInterruptException:
        pass