
# import the necessary packages
from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time
import serial

ser = serial.Serial('COM12',baudrate=9600, timeout=1)

greenLower = (37,44,100)  
greenUpper = (72, 255, 255)
redLower   = (0, 50, 145)
redUpper   = (7, 255, 255)
blueLower  = (95, 0, 100)
blueUpper  = (111, 255, 255)
yellowLower= (21, 58, 146)
yellowUpper= (35, 255, 255)

vs = VideoStream(src=1).start()
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

	mask_g = cv2.inRange(hsv, greenLower, greenUpper)
	mask_g = cv2.erode(mask_g, None, iterations=2)
	mask_g = cv2.dilate(mask_g, None, iterations=2)
	cnts_g = cv2.findContours(mask_g.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts_g = imutils.grab_contours(cnts_g)

	mask_r = cv2.inRange(hsv, redLower, redUpper)
	mask_r = cv2.erode(mask_r, None, iterations=2)
	mask_r = cv2.dilate(mask_r, None, iterations=2)
	cnts_r = cv2.findContours(mask_r.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts_r = imutils.grab_contours(cnts_r)

	mask_b = cv2.inRange(hsv, blueLower, blueUpper)
	mask_b = cv2.erode(mask_b, None, iterations=2)
	mask_b = cv2.dilate(mask_b, None, iterations=2)
	cnts_b = cv2.findContours(mask_b.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts_b = imutils.grab_contours(cnts_b)

	mask_y = cv2.inRange(hsv, yellowLower, yellowUpper)
	mask_y = cv2.erode(mask_y, None, iterations=2)
	mask_y = cv2.dilate(mask_y, None, iterations=2)
	cnts_y = cv2.findContours(mask_y.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts_y = imutils.grab_contours(cnts_y)

	
	if len(cnts_g) > 0:
		c = max(cnts_g, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		if radius > 70 and radius < 90:
			cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 0), 2)
			STM_Data = (str(center[1]).zfill(3))+(str(center[0]).zfill(3)+'g')
			ser.write(str.encode(STM_Data))
			
	elif len(cnts_r) > 0:
		c = max(cnts_r, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		if radius > 70 and radius < 90:
			cv2.circle(frame, (int(x), int(y)), int(radius),(0, 0, 255), 2)
			STM_Data = (str(center[1]).zfill(3))+(str(center[0]).zfill(3)+'r')
			ser.write(str.encode(STM_Data))
			
	elif len(cnts_b) > 0:
		c = max(cnts_b, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		if radius > 70 and radius < 90:
			cv2.circle(frame, (int(x), int(y)), int(radius),(255, 0, 0), 2)
			STM_Data = (str(center[1]).zfill(3))+(str(center[0]).zfill(3)+'b')
			ser.write(str.encode(STM_Data)) 

	elif len(cnts_y) > 0:
		c = max(cnts_y, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		if radius > 70 and radius < 90:
			cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
			STM_Data = (str(center[1]).zfill(3))+(str(center[0]).zfill(3)+'y')
			ser.write(str.encode(STM_Data)) 	

	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
 
	if key == ord("q"):
		break
	
	time.sleep(0.15)
 
vs.release()
cv2.destroyAllWindows()


