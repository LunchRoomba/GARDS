from __future__ import division
import cv2
import numpy
import scipy.misc
import time
import math

EXISTANCE_THRESHOLD = 25

def npremap(matrix, newmax):
	return matrix * (newmax/numpy.amax(matrix))

#Create obstacle avoidance slope from
def blur(image, size, iterations):
	for i in range(iterations):
		map = cv2.GaussianBlur(image, (size, size), 0)
	return numpy.maximum(map, image)

#Gradient computation from routing slopw
def gradient(image):
	var = 5
	x = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=3)
	y = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=3)
	numpy.clip(x, -var, var, out=x)
	numpy.clip(y, -var, var, out=y)
	return (x, y)

#Routing Slope Generation Using Cone Model
def routingGradientFlat(destX, destY, width, height):
	x = numpy.concatenate((numpy.arange(destX, 0, -1), numpy.arange(0, width-destX)))
	y = numpy.concatenate((numpy.arange(destY, 0, -1), numpy.arange(0, height-destY)))
	x = x**2
	y = y**2
	out = numpy.add.outer(x, y)
	return numpy.sqrt(out)

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
	width, height = map.shape
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
