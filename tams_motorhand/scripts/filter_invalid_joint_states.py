#!/usr/bin/env python

# a small helper script that sets rh_MFJ4 to 0/0/0 in the published joint_states
# This is useful when position sensing returns invalid data for the joint.
# v4hn @ 20180704

import rospy
from sensor_msgs.msg import JointState

pub= None

def filter(msg):
    global pub
    idx=msg.name.index('rh_MFJ4')
    msg.position= list(msg.position)
    msg.velocity= list(msg.velocity)
    msg.effort= list(msg.effort)
    msg.position[idx]= 0.0
    msg.velocity[idx]= 0.0
    msg.effort[idx]= 0.0
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('mfj4_joint_state_filter')
    pub= rospy.Publisher('joint_states', JointState, queue_size=1)
    sub= rospy.Subscriber('joint_states_original', JointState, filter, queue_size=1)
    rospy.spin()
