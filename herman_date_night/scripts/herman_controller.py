#!/usr/bin/env python3

import sys
import rospy
import math
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from trac_ik_python.trac_ik import IK

def main():
    rospy.init_node('joint_driver', anonymous=True)
    pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)
    rate = rospy.Rate(10)

#    ik_solver = IK("base_link", "tool0")

    while not rospy.is_shutdown():
        jt = JointTrajectory()
        jt.header.stamp = rospy.Time.now()
        jt.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        tp = JointTrajectoryPoint()
        tp.positions = [math.sin(rospy.Time.now().to_sec()),-1.5707,1.5707,0.0,1.5707,0.0]
        tp.time_from_start = rospy.Duration(nsecs=100000000)
        jt.points.append(tp)

        pub.publish(jt)
        rate.sleep()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
