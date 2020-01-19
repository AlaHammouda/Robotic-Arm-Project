
# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time
import serial

ser = serial.Serial('COM12',baudrate=9600, timeout=1)

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
greenLower = (29, 86, 6)   #(29, 86, 6)  (60, 0, 90) 
greenUpper = (64, 255, 255) # (64, 255, 255) (115, 20, 190) 

vs = VideoStream(src=0).start()
time.sleep(2.0)

# keep looping
while True:
	# grab the current frame
	frame = vs.read()
 
	# resize the frame, blur it, and convert it to the HSV
	# color space
	frame = imutils.resize(frame, width=600)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
    
       # find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	#center = None
	#center = (int(50), int(25))
	#STM_Data = (str(center[0]).zfill(3)) + (str(center[1]).zfill(3))
	#print(center)
	#ser.write(str.encode(STM_Data))
	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
		if radius > 19:
			# draw the circle and centroid on the frame,
			cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
			STM_Data = (str(center[0]).zfill(3)) + (str(center[1]).zfill(3))
			print(center)
			ser.write(str.encode(STM_Data))
			
		# loop over the set of tracked points
	
	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
 
	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break
 
vs.release()
cv2.destroyAllWindows()


