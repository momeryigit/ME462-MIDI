#!/usr/bin/env python
import sys
import time
print(sys.path)
from midibot_py import DifferentialDriveRobot as Robot
import math


'''
Welcome to the ArUco Marker Detector!

This program:
- Detects ArUco markers using OpenCV and Python
'''
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library

# Project: ArUco Marker Detector
# Date created: 12/18/2021
# Python version: 3.8
# Reference: https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/

desired_aruco_dictionary = "DICT_5X5_100"

# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
}

def main():
	"""
	Main method of the program.
	"""
	# Check that we have a valid ArUco marker
	ARUCO_DICT.get(desired_aruco_dictionary, None)


	# Load the ArUco dictionary
	print("[INFO] detecting '{}' markers...".format(
	desired_aruco_dictionary))
	this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[desired_aruco_dictionary])
	this_aruco_parameters = cv2.aruco.DetectorParameters_create()

	# Start the video stream
	cap = cv2.VideoCapture(2)
	# Set the codec to MJPG
	cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

	# Set the frame rate to 30 FPS
	cap.set(cv2.CAP_PROP_FPS, 30)
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)	
	robot = Robot(port="/dev/ttyACM0")
	robot.connect("serial")
 
	lastcommand = time.time()
 
	linear_max = 1 #in m/s
	angular_max = math.pi*3/4 #in rad/s
	
	
	while(True):

		# Capture frame-by-frame
		# This method returns True/False as well
		# as the video frame.
		#capture by 480, 640
		ret, frame = cap.read()
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		# Detect ArUco markers in the video frame
		(corners, ids, rejected) = cv2.aruco.detectMarkers(
		gray, this_aruco_dictionary, parameters=this_aruco_parameters)

		(h,w) = frame.shape[:2]

		# Check that at least one ArUco marker was detected
		if len(corners) > 0:
			# Flatten the ArUco IDs list
			ids = ids.flatten()

			# Loop over the detected ArUco corners
			for (marker_corner, marker_id) in zip(corners, ids):

				# Extract the marker corners
				corners = marker_corner.reshape((4, 2))
				(top_left, top_right, bottom_right, bottom_left) = corners

				# Convert the (x,y) coordinate pairs to integers
				top_right = (int(top_right[0]), int(top_right[1]))
				bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
				bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
				top_left = (int(top_left[0]), int(top_left[1]))


				# Calculate and draw the center of the ArUco marker
				center_x = int((top_left[0] + bottom_right[0]) / 2.0)
				center_y = int((top_left[1] + bottom_right[1]) / 2.0)
				cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)

				#draw x distance between the center of the marker and the center of the frame
				error = -1*(w/2 - center_x)
				cv2.putText(frame, str(error), (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
				#calculate area of the marker
				area = cv2.contourArea(corners)
				cv2.putText(frame, str(area), (top_left[0], top_left[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

				#linearly map the area to the linear velocity inverse proportional
				linear_vel = linear_max - (area/30000)*linear_max
				#angularly map the error to the angular velocity
				angular_vel = error/w*angular_max

				
				if time.time() - lastcommand > 0.2:
					#robot movement
					f_r,f_l =robot.send_twist(linear_vel, angular_vel)
					print("linear vel: ", linear_vel, "angular vel: ", angular_vel, "f_r: ", f_r, "f_l: ", f_l)
					lastcommand = time.time()
				
				

				# Draw the ArUco marker ID on the video frame
				# The ID is always located at the top_left of the ArUco marker
				cv2.putText(frame, str(marker_id), 
				(top_left[0], top_left[1] - 15),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)

		elif len(corners) == 0 and time.time() - lastcommand > 0.5:
			robot.stop()
			print("STOP!")
			lastcommand = time.time()
   
		# Display the resulting frame
		cv2.imshow('frame',frame)

		# If "q" is pressed on the keyboard, 
		# exit this loop
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	# Close down the video stream
	cap.release()
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()