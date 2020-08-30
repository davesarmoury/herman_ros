#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def callback(msg):
    out_file = open("points.xyz", 'a')

    for point in pc2.read_points(msg, skip_nans=True):
            pt_x = str(point[0])
            pt_y = str(point[1])
            pt_z = str(point[2])
            out_file.write(pt_x + " " + pt_y + " " + pt_z + "\n")

    out_file.close()

def listener():
    out_file = open("points.xyz", 'w')
    out_file.close()

    rospy.init_node('cloud_to_points', anonymous=True)

    rospy.Subscriber("/outlier/output", PointCloud2, callback)
    #rospy.Subscriber("/cloud", PointCloud2, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
