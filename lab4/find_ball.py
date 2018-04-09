#!/usr/bin/env python3

import cv2
import sys
import copy

import numpy as np

try:
	from PIL import Image, ImageDraw, ImageFont
except ImportError:
	sys.exit('install Pillow to run this code')


def find_balls(img, thresh):
	balls = cv2.HoughCircles(img, method=cv2.HOUGH_GRADIENT, dp=1, minDist=20, 
                            param1=100, param2=thresh)

	return [] if balls is None else balls[0]

def find_ball(img=None, debug=False, min_thresh=30, max_thresh=100, blurred=None):
	"""Find the ball in an image.
		
		Arguments:
		img -- the image
		debug -- an optional argument which can be used to control whether
				debugging information is displayed.
		
		Returns [x, y, radius] of the ball, and [0,0,0] or None if no ball is found.
	"""

	# End condition
	if max_thresh - min_thresh <= 0:
		return None

	blurred = blurred if blurred is not None else cv2.GaussianBlur(img,(5,5),0)

	thresh = (max_thresh + min_thresh) // 2
	balls = find_balls(blurred, thresh)

	if len(balls) == 0:
		return find_ball(blurred=blurred, debug=debug, min_thresh=min_thresh, max_thresh=thresh)
	elif len(balls) == 1:
		if debug:
			print(thresh)
		return balls[0]
	else:
		next = find_ball(blurred=blurred, debug=debug, min_thresh=thresh, max_thresh=max_thresh)
		if next is None:
			return balls[0]
		else:
			return next


def display_circles(opencv_image, circles, best=None):
	"""Display a copy of the image with superimposed circles.
		
	   Provided for debugging purposes, feel free to edit as needed.
	   
	   Arguments:
		opencv_image -- the image
		circles -- list of circles, each specified as [x,y,radius]
		best -- an optional argument which may specify a single circle that will
				be drawn in a different color.  Meant to be used to help show which
				circle is ranked as best if there are multiple candidates.
		
	"""
	#make a copy of the image to draw on
	circle_image = copy.deepcopy(opencv_image)
	circle_image = cv2.cvtColor(circle_image, cv2.COLOR_GRAY2RGB, circle_image)
	
	color = 255
	for c in circles:
		# draw the outer circle
		cv2.circle(circle_image,(c[0],c[1]),c[2],(color, color, 0),1)
		# draw the center of the circle
		cv2.circle(circle_image,(c[0],c[1]),2,(0,255,255),1)
	
	#highlight the best circle in a different color
	if best is not None:
		# draw the outer circle
		cv2.circle(circle_image,(best[0],best[1]),best[2],(0,0,255),2)
		# draw the center of the circle
		cv2.circle(circle_image,(best[0],best[1]),2,(0,0,255),3) 
		# write coords
		cv2.putText(circle_image,str(best),(best[0],best[1]),cv2.FONT_HERSHEY_SIMPLEX,
					.5,(255,255,255),1,cv2.LINE_AA)
		
	
	#display the image
	pil_image = Image.fromarray(circle_image)
	pil_image.show()

if __name__ == "__main__":
	opencv_image = cv2.imread("./imgs/test%s.bmp" % (sys.argv[1]), cv2.COLOR_GRAY2RGB)
	balls = find_ball(opencv_image, debug=True)
	display_circles(opencv_image, [], balls)
