#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from std_msgs.msg import Float32, Float64 #####
from mavros.msg import ActuatorControl
from autopilot import *
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

kunalorme = makeAutopilot()

current_controls = [0, 0, 0, 0]

def mast_autopilot_talker():
    pub = rospy.Publisher("/mavros/actuator_control", ActuatorControl, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        publish_controls(pub)
    rate.sleep()


def publish_controls(pub):
    msg = ActuatorControl()
    msg.header = Header()
    msg.header.frame_id = "base_footprint"
    msg.header.stamp = rospy.Time.now()
    msg.group_mix = 0
    # [roll pitch yaw throttle flaps spoilers brakes gear]
    msg.controls = current_controls + [0] * 4
    # rospy.loginfo('%s',str(msg.controls))
    pub.publish(msg)


def on_imu_msg(data):
    global current_controls

    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z
    w = data.orientation.w
    phi, theta, psi = euler_from_quaternion([x,y,z,w])

    p = data.angular_velocity.x
    q = data.angular_velocity.y
    r = data.angular_velocity.z

    kunalorme.process_inputs['phi'] = phi
    kunalorme.process_inputs['theta'] = theta
    # kunalorme.process_inputs['psi'] = psi

    kunalorme.process_inputs['p'] = p
    kunalorme.process_inputs['q'] = q
    kunalorme.process_inputs['r'] = r

    current_controls = kunalorme.update()




def on_local_position_msg(data):
    #global current_controls

    # h = data.position.z
    h = 100
    Va = 30

    kunalorme.process_inputs['h'] = h
    kunalorme.process_inputs['Va'] = Va

    #current_controls = kunalorme.update()

def on_compass_hdg_msg(data):
    # global current_controls

    # kunalorme.process_inputs['chi'] = data
    kunalorme.process_inputs['chi'] = 0

    # current_controls = kunalorme.update()

def mast_autopilot_listener():
    rospy.Subscriber("/mavros/imu/data", Imu, on_imu_msg)
    rospy.Subscriber("/mavros/local_position/local", PoseStamped, on_local_position_msg)
    rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, on_local_position_msg)
    rospy.Subscriber("/mavros/vfr_hud", Float32, on_local_position_msg) ####
    rospy.spin()


if __name__ == '__main__':
    kunalorme.commanded['h'] = 100
    kunalorme.commanded['Va'] = 30
    kunalorme.commanded['chi'] = 0

    rospy.init_node('mast_autopilot', anonymous=True)
    mast_autopilot_listener()
    mast_autopilot_talker()

