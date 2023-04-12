#!/usr/bin/env python

# moves hand to an open and safe joint configuration

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('spread_hand')
fingers_pub = rospy.Publisher('/hand/rh_trajectory_controller/command', JointTrajectory, queue_size=1, latch=True)
wrist_pub = rospy.Publisher('/hand/rh_wr_trajectory_controller/command', JointTrajectory, queue_size=1, latch=True)

traj = JointTrajectory()
traj.joint_names = ['rh_FFJ4', 'rh_FFJ3', 'rh_FFJ2', 'rh_FFJ1', 'rh_LFJ5', 'rh_LFJ4', 'rh_LFJ3', 'rh_LFJ2', 'rh_LFJ1', 'rh_MFJ4', 'rh_MFJ3', 'rh_MFJ2', 'rh_MFJ1', 'rh_RFJ4', 'rh_RFJ3', 'rh_RFJ2', 'rh_RFJ1', 'rh_THJ5', 'rh_THJ4', 'rh_THJ3', 'rh_THJ2', 'rh_THJ1']
traj.points.append(JointTrajectoryPoint())
traj.points[0].positions = [
    -0.23131151280059523,
    0.007094041984987078,
    0.02542655924667998,
    0.349065850399,
    0.020020591171226638,
    -0.31699402670752413,
    0.028414672906931473,
    0.08222580050009387,
    0.349065850399,
    0.008929532157657268,
    0.002924555968379014,
    0.010452066813274026,
    0.349065850399,
    -0.11378487918959583,
    0.014028701132086206,
    0.0011093194398269044,
    0.349065850399,
    -0.4480210297267876,
    0.048130900509343995,
    0.0018684990049854181,
    -0.11587408526722277,
    0.349065850399]
traj.points[0].time_from_start = rospy.Duration(1.0)
fingers_pub.publish(traj)

traj = JointTrajectory()
traj.joint_names = ['rh_WRJ2', 'rh_WRJ1']
traj.points.append(JointTrajectoryPoint())
traj.points[0].positions = [
    -0.07571641056445733,
    -0.05002482770016008]
traj.points[0].time_from_start = rospy.Duration(1.0)
wrist_pub.publish(traj)

rospy.sleep(5.0)
