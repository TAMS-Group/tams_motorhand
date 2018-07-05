#!/usr/bin/env python

# by Michael Goerner @ 20161011

# This script publishes 0 joint states for all joints of the right hand and creates a stub action server
# This is useful when the hand is not needed, but required in the Trixi configs

import rospy
import sensor_msgs.msg

import actionlib
import control_msgs.msg

class HandStub:
  def __init__(self):
    self.action= actionlib.SimpleActionServer("rh_trajectory_controller/follow_joint_trajectory", control_msgs.msg.FollowJointTrajectoryAction, execute_cb= self.action_cb, auto_start= False)
    self.action.start()

    self.joint_pub= rospy.Publisher('joint_states', sensor_msgs.msg.JointState, queue_size= 1)

    joints= ['rh_WRJ2', 'rh_WRJ1', 'rh_FFJ4', 'rh_FFJ3', 'rh_FFJ2', 'rh_FFJ1', 'rh_LFJ5', 'rh_LFJ4', 'rh_LFJ3', 'rh_LFJ2', 'rh_LFJ1', 'rh_MFJ4', 'rh_MFJ3', 'rh_MFJ2', 'rh_MFJ1', 'rh_RFJ4', 'rh_RFJ3', 'rh_RFJ2', 'rh_RFJ1', 'rh_THJ5', 'rh_THJ4', 'rh_THJ3', 'rh_THJ2', 'rh_THJ1']

    self.js= sensor_msgs.msg.JointState(
      name= joints,
      position= [0.0]*len(joints),
      velocity= [0.0]*len(joints),
      effort= [0.0]*len(joints))

    # J1 is a fixed joint with the BioTac sensors
    for i in range(len(joints)):
      if "J1" in joints[i]:
        self.js.position[i]= 0.35

  def action_cb(self, goal):
    self.action.set_aborted(None, "This is a stub that is supposed to fail")

  def run(self):
    r= rospy.Rate(50)
    while not(rospy.is_shutdown()):
      self.js.header.stamp= rospy.Time.now()
      self.joint_pub.publish(self.js)
      r.sleep()


if __name__ == '__main__':
  rospy.init_node('hand_stub')

  stub= HandStub()

  stub.run()
