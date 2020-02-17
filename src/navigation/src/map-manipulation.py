from __future__ import division
import cv2
import numpy
import scipy.misc
import time
import math
import rospy as ros
from nav_msgs.msg import _OccupancyGrid as OccupancyGrid

EXISTANCE_THRESHOLD = 25

def npremap(matrix, newmax):
	return matrix * (newmax/numpy.amax(matrix))

def gridflip(grid):
	return 100-grid

#Create obstacle avoidance slope from occupancy grid
def blur(image, size, iterations):
	for i in range(iterations):
		map = cv2.GaussianBlur(image, (size, size), 0)
	return numpy.maximum(map, image)

#Gradient computation from routing slope
def gradient(image):
	x = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=9)
	y = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=9)
	numpy.clip(x, -5, 5, out=x)
	numpy.clip(y, -5, 5, out=y)
	return (x, y)

#D* based routing slope
def dStarFillPrep(destX, destY, map):
	width, height = map.shape
	grad = numpy.zeros((width, height))
	grad = grad - map - 1
	grad[destX, destY] = 0
	frontline = [(destX, destY)]
	dStarFill(grad, frontline)
	grad = numpy.clip(grad, 0, width*height)
	return grad

def dStarFill(grad, frontline):
	q = range(-1, 2)
	r = [((x, y), math.sqrt(x**2+y**2)) for x in q for y in q if x != 0 or y != 0]
	width, height = grad.shape
	while len(frontline) > 0:
		temp = []
		for spot in frontline:
			x, y = spot
			for (xmod, ymod), dist in r:
				cost = grad[x,y] + dist
				if x<width-1 and y<height-1 and x>0 and y>0:
					next = grad[x+xmod, y+ymod]
					if next > cost or (next <= 0 and next > -(EXISTANCE_THRESHOLD)-1):
						grad[x+xmod, y+ymod] = cost
						temp.append((x+xmod, y+ymod))
			frontline.remove(spot)
		frontline = frontline+temp
		scipy.misc.imsave('route.jpg', grad)
	return

def updatedSlope(grid, x, y):
	map = gridflip(numpy.array(grid))
	blur = blur(map, 70, 2)
	#recieve input from database
	x=map.shape[0]//2
	y=map.shape[1]//2
	return dStarFillPrep(x, y, map) + blur

def runUpdates(occupancyGrid):
	return

def main():
    rospy.init_node("map-modification")
    rospy.Subscriber('/rtabmap/proj_map', OccupancyGrid, runUpdates)
    rospy.spin()

if __name__ == '__main__':
    main()
'''
def main(map, x, y):
	width, height = map.shape
	route = dStarFillPrep(x, y, map)
	blurred = blur(map, 201, 3)
	map = npremap(map, numpy.amax(route))
	slope = blurred+route
	#map = numpy.maximum(map, slope)
	map = slope
	x, y = gradient(map)
	scipy.misc.imsave('blurred.jpg', blurred)
	scipy.misc.imsave('final.jpg', map)
	scipy.misc.imsave('x.jpg', x)
	scipy.misc.imsave('y.jpg', y)
	scipy.misc.imsave('both.jpg', x+y)

	return (x,y)


map = cv2.imread("map.jpg")
map = cv2.cvtColor(map, cv2.COLOR_BGR2GRAY)
map = npremap(map, 100)
main(map, map.shape[0]//2, map.shape[1]//2)
'''
