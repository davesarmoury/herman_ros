#!/usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose
from positions import *

# Main Function #
if __name__=='__main__':
    # Makes a ROS node #
    rospy.init_node('moveit_py', anonymous=True)

    # Gets a handle on the robot that MoveIt is controlling
    robot = RobotCommander()
    rospy.sleep(1)

    # Picks a group called "manipulator" to work with #
    arm = robot.get_group("manipulator")
    pick_rubik(arm, "back")
    turn_rubik(arm, "clockwise")
    place_rubik(arm, "back")
