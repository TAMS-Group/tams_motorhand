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
    prefix= rospy.get_param("prefix", default="rh_")
    self.action= actionlib.SimpleActionServer(prefix+"trajectory_controller/follow_joint_trajectory", control_msgs.msg.FollowJointTrajectoryAction, execute_cb= self.action_cb, auto_start= False)
    self.action.start()

    self.joint_pub= rospy.Publisher('joint_states', sensor_msgs.msg.JointState, queue_size= 1)

    joints= ['WRJ2', 'WRJ1', 'FFJ4', 'FFJ3', 'FFJ2', 'FFJ1', 'LFJ5', 'LFJ4', 'LFJ3', 'LFJ2', 'LFJ1', 'MFJ4', 'MFJ3', 'MFJ2', 'MFJ1', 'RFJ4', 'RFJ3', 'RFJ2', 'RFJ1', 'THJ5', 'THJ4', 'THJ3', 'THJ2', 'THJ1']
    joints= [prefix + j for j in joints]

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
    self.action.set_aborted(None, "This is a stub. Execution will always fail.")

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
