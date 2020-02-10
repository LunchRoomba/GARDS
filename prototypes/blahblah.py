import cv2 as cv
import numpy as np
import random as rand
import time

def floodfill(x, y, val, pic):
	h,w = pic.shape[:2]
	if val > 0 and x < w-1 and y<h-1 and y>0+1 and x>0+1:
		pic[x][y]=255
		case = rand.randint(0, 3)
		if case == 0:
			pic=floodfill(x-1,y,val-1, pic)
		elif case == 1:
			pic=floodfill(x+1,y,val-1, pic)
		elif case == 2:
			pic=floodfill(x,y-1,val-1, pic)
		elif case == 3:
			pic=floodfill(x,y+1,val-1, pic)
	return pic

def main(height, width):
	blah = True
	height = width
	zeros = np.zeros((height, width))
	while True:
		pic = np.zeros((height, width))
		pic = floodfill(int(width/2), int(height/2),rand.randint(0, min(height, width)/2), pic)
		if blah:
			cv.imshow("asdf", pic)
		else:
			cv.imshow("asdf", zeros)
		blah= not blah
		time.sleep(0.1)
	return

main(200, 200)
