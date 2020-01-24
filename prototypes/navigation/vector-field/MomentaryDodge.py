#asdfasdfkasgfkhlj
from __future__ import division
import cv2
import scipy.misc
import time
import math

import rospy
import numpy as np

from ac_msgs.msg import drive_params
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

from lidar_utils import get_distance_array

#if angle = 50: searches area between -50 and 50
def histogram(angle, step, lidar, threshold):
	ret = []
	for i in range(0, angle*2, step):
		i=i-angle
		sum = get_distance_array(lidar, i, i+step).sum()
		if sum>threshold:
			dir = ((i+i+step)/2)/360
			ret.append([sum, dir, math.abs(dir)])
	ret.sort(key = lambda x:x[0], reverse=True)
	return ret

if __name__ == '__main__':
    rospy.init_node('anticrash', anonymous=True)
    driveMod_pub = rospy.Publisher('drive_mod', Float64, queue_size=1)
    driveMod = Float64(0.0)
 # Obstacle based
    while not rospy.core.is_shutdown_requested():
        laser_data = rospy.client.wait_for_message('scan', LaserScan)

		hist = histogram(180, 10, laser_data, 1)

		driveMod = Float64(min(hist, key = lambda x:x[2])[1])

     	driveMod_pub.publish(driveMod)
