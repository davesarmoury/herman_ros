#!/usr/bin/env python
import rospy
from herman_rubiks.msg import StringArray
from std_msgs.msg import String
from rubik_solver import utils

pub = None

def callback(data):
    global pub

    if solver_type:
        rospy.loginfo("Solving with Kociemba")
        msg = StringArray()
        steps = utils.solve(data.data, 'Kociemba')
        for s in steps:
            msg.steps.append(str(s))
        pub.publish(msg)
        rospy.loginfo("Done")
    else:
        rospy.logerr("Invalid solver type")

def solver():
    global pub

    pub = rospy.Publisher('cube_steps', StringArray, queue_size=10)
    rospy.Subscriber("cube_configuration", String, callback)
    rospy.init_node('solver', anonymous=True)

    rospy.loginfo("Started ( Kociemba )")

    rospy.spin()

if __name__ == '__main__':
    try:
        solver()
    except rospy.ROSInterruptException:
        pass
