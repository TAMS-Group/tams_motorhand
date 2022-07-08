#!/usr/bin/env python

import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

import sensor_msgs.msg

rospy.init_node('detect_unresponsive_hand')

diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size= 1)

latest= None
def cb(msg):
  global latest
  latest = msg.header.stamp

test_sub = rospy.Subscriber('joint_states', sensor_msgs.msg.JointState, cb, queue_size= 1)

r= rospy.Rate(1)

da= DiagnosticArray()
da.status.append(DiagnosticStatus())

while not rospy.is_shutdown():
  da.header.stamp = rospy.Time.now()
  da.status[0].name = "ShadowHand Monitor"
  if latest is not None and latest + rospy.Duration(1.0) < da.header.stamp:
    da.status[0].level = DiagnosticStatus.ERROR
    da.status[0].message = "ShadowHand driver shutdown (restart the driver to fix)"
  else:
    da.status[0].level = DiagnosticStatus.OK
    da.status[0].message = "ShadowHand reports current joint values"
  diag_pub.publish(da)
  r.sleep()
