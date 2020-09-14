#!/usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose
from positions import *
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import math

read_colours = False
past_reads = []
colours = {}
colours["R"] = [255,0,0]
colours["G"] = [0,255,0]
colours["B"] = [0,0,255]
colours["W"] = [255,255,255]
colours["O"] = [255,128,0]
colours["Y"] = [255,255,0]

def getEuclideanColour(rgb):
    lowest = 999999
    colour = "X"
    for ckey in colours.keys():
        c = colours[ckey]
        distance = math.sqrt((rgb[0]-c[0])*(rgb[0]-c[0]) + (rgb[1]-c[1])*(rgb[1]-c[1]) + (rgb[2]-c[2])*(rgb[2]-c[2]))
        if distance < lowest:
            lowest = distance
            colour = ckey

    return colour

def callback(msg):
    global read_colours, past_reads

    if read_colours:
        br = CvBridge()
        img = br.imgmsg_to_cv2(msg)

        squares = []
        squares.append(getEuclideanColour(img[170,460]))
        squares.append(getEuclideanColour(img[170,650]))
        squares.append(getEuclideanColour(img[170,840]))
        squares.append(getEuclideanColour(img[360,460]))
        squares.append(getEuclideanColour(img[360,650]))
        squares.append(getEuclideanColour(img[360,840]))
        squares.append(getEuclideanColour(img[550,460]))
        squares.append(getEuclideanColour(img[550,650]))
        squares.append(getEuclideanColour(img[550,840]))

        str = ""
        for s in squares:
            str = str + s
        print(str)
        past_reads.append(squares)
        read_colours = False

def read(arm, index):
    global read_colours
    arm.go(camera_sides[index])
    arm.stop()
    r = rospy.Rate(10)

    read_colours = True
    while not rospy.is_shutdown():
        if not read_colours:
            break
        r.sleep()

def reads_to_cube():
    global past_reads

    cube_string = ""
    # Top #
    cube_string += past_reads[3][0]
    cube_string += past_reads[3][1]
    cube_string += past_reads[3][2]
    cube_string += past_reads[8][5]
    cube_string += "y"
    cube_string += past_reads[3][4]
    cube_string += past_reads[3][5]
    cube_string += past_reads[3][6]
    cube_string += past_reads[3][8]
    # Left #
    cube_string += past_reads[5][0]
    cube_string += past_reads[5][1]
    cube_string += past_reads[5][2]
    cube_string += past_reads[5][3]
    cube_string += "b"
    cube_string += past_reads[5][4]
    cube_string += past_reads[5][5]
    cube_string += past_reads[5][6]
    cube_string += past_reads[5][8]
    # Front #
    cube_string += past_reads[2][0]
    cube_string += past_reads[2][1]
    cube_string += past_reads[2][2]
    cube_string += past_reads[9][5]
    cube_string += "r"
    cube_string += past_reads[2][4]
    cube_string += past_reads[2][5]
    cube_string += past_reads[2][6]
    cube_string += past_reads[2][8]
    # Right #
    cube_string += past_reads[0][0]
    cube_string += past_reads[0][1]
    cube_string += past_reads[0][2]
    cube_string += past_reads[0][3]
    cube_string += "g"
    cube_string += past_reads[0][4]
    cube_string += past_reads[0][5]
    cube_string += past_reads[0][6]
    cube_string += past_reads[0][8]
    # Back #
    cube_string += past_reads[4][0]
    cube_string += past_reads[4][1]
    cube_string += past_reads[4][2]
    cube_string += past_reads[7][5]
    cube_string += "o"
    cube_string += past_reads[4][4]
    cube_string += past_reads[4][5]
    cube_string += past_reads[4][6]
    cube_string += past_reads[4][8]
    # Bottom #
    cube_string += past_reads[1][0]
    cube_string += past_reads[1][1]
    cube_string += past_reads[1][2]
    cube_string += past_reads[6][5]
    cube_string += "w"
    cube_string += past_reads[1][4]
    cube_string += past_reads[1][5]
    cube_string += past_reads[1][6]
    cube_string += past_reads[1][8]

    return cube_string

def commander():
    global past_reads

    rospy.init_node('rubik_camera', anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    pub = rospy.Publisher("cube_configuration", String, queue_size=1)

    # Gets a handle on the robot that MoveIt is controlling
    robot = RobotCommander()
    rospy.sleep(1)

    # Picks a group called "manipulator" to work with #
    arm = robot.get_group("manipulator")
    arm.set_max_velocity_scaling_factor(1.0)
    arm.set_goal_position_tolerance(0.0001)
    arm.set_goal_orientation_tolerance(0.005)

    go_home(arm)
    pick_rubik(arm, robot, "right")
    go_home(arm)
    for i in range(5):
        read(arm, i)
    go_home(arm)
    place_rubik(arm, robot, "right")
    go_home(arm)
    pick_rubik(arm, robot, "left")
    go_home(arm)
    for i in range(5):
        read(arm, i)
    go_home(arm)
    place_rubik(arm, robot, "left")
    go_home(arm)
    print(str(past_reads))
    pub.publish(reads_to_cube())
    rospy.sleep(1)

if __name__=='__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass
