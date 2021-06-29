#!/usr/bin/env python

# a small helper script that sets invalid joints to 0/0/0 in the published joint_states
# This is useful when position sensing returns invalid data for a joint.
# v4hn @ 20180704 for mfj4
# v4hn @ 20210629 use for lfj2

import rospy
from sensor_msgs.msg import JointState

pub= None

INVALID_JOINTS= [
#    'rh_MFJ4',
    'rh_LFJ2',
]

def zero_invalid_joints(msg):
    global pub

    msg.position= list(msg.position)
    msg.velocity= list(msg.velocity)
    msg.effort= list(msg.effort)

    for j in INVALID_JOINTS:
        idx=msg.name.index(j)
        msg.position[idx]= 0.0
        msg.velocity[idx]= 0.0
        msg.effort[idx]= 0.0

    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('invalid_joint_state_filter')
    pub= rospy.Publisher('joint_states', JointState, queue_size=1)
    sub= rospy.Subscriber('joint_states_original', JointState, zero_invalid_joints, queue_size=1)
    rospy.spin()
