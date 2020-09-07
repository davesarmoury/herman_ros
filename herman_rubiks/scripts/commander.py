#!/usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose
from positions import *
from herman_rubiks.msg import StringArray

solution = []

def callback(msg):
    global solution
    rospy.loginfo("Received new solution %s", str(msg.steps))
    solution = []
    for s in msg.steps:
        solution.append(s)

def commander():
    global solution
    # Makes a ROS node #
    rospy.init_node('rubik_commander', anonymous=True)

    # Gets a handle on the robot that MoveIt is controlling
    robot = RobotCommander()
    rospy.sleep(1)

    rospy.Subscriber('cube_steps', StringArray, callback)

    # Picks a group called "manipulator" to work with #
    arm = robot.get_group("manipulator")
    arm.set_max_velocity_scaling_factor(0.5)
    arm.set_goal_position_tolerance(0.0001)
    arm.set_goal_orientation_tolerance(0.005)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        if len(solution) > 0:
            for move in solution:
                print("### " + str(move) + " ###")
                ##### PICK #####
                go_home(arm)

                if "F" in move:
                    front_to_right(arm, robot)
                    go_home(arm)
                    pick_rubik(arm, robot, "right")
                if "B" in move:
                    pick_rubik(arm, robot, "back")
                if "L" in move:
                    pick_rubik(arm, robot, "left")
                if "R" in move:
                    pick_rubik(arm, robot, "right")
                if "U" in move:
                    expose_up_down(arm, robot)
                    go_home(arm)
                    pick_rubik(arm, robot, "right")
                if "D" in move:
                    expose_up_down(arm, robot)
                    go_home(arm)
                    pick_rubik(arm, robot, "left")

                go_home(arm)
                ##### TURN #####
                if "'" in move:
                    turn_rubik(arm, robot, "counter")
                elif "2" in move:
                    turn_rubik(arm, robot, "180")
                else:
                    turn_rubik(arm, robot, "clockwise")

                go_home(arm)
                ##### PLACE
                if "F" in move:
                    place_rubik(arm, robot, "right")
                    go_home(arm)
                    right_to_front(arm, robot)
                if "B" in move:
                    place_rubik(arm, robot, "back")
                if "L" in move:
                    place_rubik(arm, robot, "left")
                if "R" in move:
                    place_rubik(arm, robot, "right")
                if "U" in move:
                    place_rubik(arm, robot, "right")
                    go_home(arm)
                    reset_up_down(arm, robot)
                if "D" in move:
                    place_rubik(arm, robot, "left")
                    go_home(arm)
                    reset_up_down(arm, robot)

                go_home(arm)

            solution = []
        rate.sleep()

if __name__=='__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass
