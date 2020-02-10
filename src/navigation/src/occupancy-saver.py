from __future__ import division
import cv2
import numpy
import scipy.misc
import time
import math
import rospy
from nav_msgs.msg import OccupancyGrid

EXISTANCE_THRESHOLD = 25

def callback(msg):
	scipy.misc.imsave('occupancygrid.jpg', msg.data)

def main():
	rospy.init_node("insertnamehere")
	sub = rospy.Subscriber("/rtabmap/proj_map", OccupancyGrid, callback)
	rospy.spin()