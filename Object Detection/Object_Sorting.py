
# import the necessary packages.
from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time
import serial

# configure the serial connections
ser = serial.Serial('COM12',baudrate=9600, timeout=1)

# define the lower and upper boundaries of the "green"
greenLower = (45,44,100)  
greenUpper = (85, 255, 255)

# define the lower and upper boundaries of the "red"
redLower   = (0, 50, 145)
redUpper   = (7, 255, 255)

# define the lower and upper boundaries of the "blue"
blueLower  = (95, 170, 123)
blueUpper  = (111, 255, 255)

# define the lower and upper boundaries of the "yellow"
yellowLower= (21, 90, 100)
yellowUpper= (35, 255, 255)


def Grab_Contours(hsv_space,colorLower,colorUpper):
	# construct a mask for the color , then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv_space, colorLower, colorUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	# find contours in the mask	
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	return cnts

def Process_Color(cnts,color,color_key):
	c = max(cnts, key=cv2.contourArea)
	((x, y), radius) = cv2.minEnclosingCircle(c)
	M = cv2.moments(c)
	center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius is within 
	if radius > 70 and radius < 90:
                # draw the circle
		cv2.circle(frame, (int(x), int(y)), int(radius),color, 2)
		STM_Data = (str(center[1]).zfill(3))+(str(center[0]).zfill(3)+color_key)
		ser.write(str.encode(STM_Data))
		return 1
	else:
		return 0

object_detected=0

# grab the reference to the webcam
# allow the camera to warm up
vs = VideoStream(src=1).start()
time.sleep(2.0)

# keep looping
while True:
	# grab the current frame
	frame = vs.read()

	# resize the frame, blur it, and convert it to the HSV color space
	frame = imutils.resize(frame, width=600)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)    

        # get contours for each color  
	cnts_g = Grab_Contours(hsv,greenLower,greenUpper)
	cnts_r = Grab_Contours(hsv,redLower,redUpper)
	cnts_b = Grab_Contours(hsv,blueLower,blueUpper)
	cnts_y = Grab_Contours(hsv,yellowLower,yellowUpper)

	object_detected = 0

	# only proceed if at least one contour was found for green 
	if len(cnts_g) > 0:
			object_detected = Process_Color(cnts_g,(0,255,0),'g')
	# only proceed if at least one contour was found for red 		
	if len(cnts_r) > 0 and not(object_detected):
			object_detected = Process_Color(cnts_r,(0,0,255),'r')
	# only proceed if at least one contour was found for blue 		
	if len(cnts_b) > 0 and not(object_detected):
			object_detected = Process_Color(cnts_b,(255,0,0),'b')
        # only proceed if at least one contour was found for yellow 
	if len(cnts_y) > 0 and not(object_detected):
			object_detected = Process_Color(cnts_y,(0,255,255),'y')

        # show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
 
	if key == ord("q"):
		break

	# process data every 150 ms 
	time.sleep(0.15)

# release the camera
# close all windows
vs.release()
cv2.destroyAllWindows()

