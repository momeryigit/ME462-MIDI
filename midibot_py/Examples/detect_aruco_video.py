#!/usr/bin/env python
import cv2
import numpy as np
import os
import sys
import time
import math
import subprocess
from midibot_py import DifferentialDriveRobot as Robot

# Constants
DESIRED_ARUCO_DICTIONARY = "DICT_5X5_100"
PHOTOS_FOLDER = "captured_photos"
PHOTOS_LIMIT = 6
CAPTURE_INTERVAL = 1  # seconds
COMMAND_INTERVAL = 0.1  # seconds
STOP_COMMAND_INTERVAL = 0.5  # seconds
LINEAR_MAX = 1  # m/s
ANGULAR_MAX = math.pi * 7 / 4  # rad/s
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
CAMERA_WARM_UP_TIME = 2  # seconds

# ArUco Dictionaries
ARUCO_DICT = {
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
}

# Function to set exposure time absolute
def set_exposure_time_absolute(device, exposure_time):
    try:
        subprocess.run(['v4l2-ctl', '-d', device, '--set-ctrl', f'exposure_time_absolute={exposure_time}'], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Failed to set exposure time absolute: {e}")
        
def setup_camera():
    """Set up the camera settings."""
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("Error: Could not open video device.")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    set_exposure_time_absolute('/dev/video0', 100) #Dropping Exposure for better capture
    cap.set(cv2.CAP_PROP_AUTO_WB, 1)
    time.sleep(CAMERA_WARM_UP_TIME)
    return cap

def detect_and_process_markers(frame, aruco_dict, aruco_params):
    """Detect ArUco markers and process them."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    return corners, ids

def draw_markers_and_calculate_velocity(frame, corners, ids, width):
    """Draw markers on the frame and calculate velocity for robot movement."""
    if ids is not None:
        for marker_corner, marker_id in zip(corners, ids.flatten()):
            corners = marker_corner.reshape((4, 2))
            top_left, top_right, bottom_right, bottom_left = corners.astype(int)
            center_x, center_y = np.mean(corners, axis=0).astype(int)
            error = width / 2 - center_x
            area = cv2.contourArea(corners)
            linear_vel = LINEAR_MAX - (area / 30000) * LINEAR_MAX
            angular_vel = error / width * ANGULAR_MAX * -1
            draw_marker(frame, top_left, top_right, bottom_right, bottom_left, center_x, center_y, marker_id, error, area)
            return linear_vel, angular_vel, area
    return 0, 0

def draw_marker(frame, top_left, top_right, bottom_right, bottom_left, center_x, center_y, marker_id, error, area):
    """Draw the bounding box, center, error, and area of the ArUco marker."""
    cv2.polylines(frame, [np.array([top_left, top_right, bottom_right, bottom_left])], True, (0, 255, 0), 2)
    cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
    cv2.putText(frame, f"Error: {error}", (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, f"Area: {area}", (top_left[0], top_left[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, str(marker_id), (top_left[0], top_left[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

def main():
	if not os.path.exists(PHOTOS_FOLDER):
		os.makedirs(PHOTOS_FOLDER)

	print(f"[INFO] Detecting '{DESIRED_ARUCO_DICTIONARY}' markers...")
	aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[DESIRED_ARUCO_DICTIONARY])
	aruco_params = cv2.aruco.DetectorParameters_create()
	cap = setup_camera()

	robot = Robot(port="/dev/ttyACM0")
	robot.connect("serial")

	last_command_time = time.time()
	photos_taken = 0
	last_photo_time = time.time()

	while True:
		ret, frame = cap.read()
		if not ret:
			print("Failed to capture frame")
			break

		# Detect markers and move the robot if the specified interval has passed
		if time.time() - last_command_time > COMMAND_INTERVAL:
			corners, ids = detect_and_process_markers(frame, aruco_dict, aruco_params)
			if ids is not None:
				linear_vel, angular_vel, area = draw_markers_and_calculate_velocity(frame, corners, ids, FRAME_WIDTH)
				robot.send_twist(linear_vel, angular_vel)
				print(f"Linear velocity: {linear_vel}, Angular velocity: {angular_vel}, Area: {area}")
			else:
				robot.stop()          
                
			last_command_time = time.time()

		# Take a photo at the specified interval, independent of robot movement
		if time.time() - last_photo_time > CAPTURE_INTERVAL and photos_taken < PHOTOS_LIMIT:
			photo_path = os.path.join(PHOTOS_FOLDER, f"photo_{photos_taken + 1}.jpg")
			cv2.imwrite(photo_path, frame)
			photos_taken += 1
			print(f"Photo {photos_taken} saved at {photo_path}")
			last_photo_time = time.time()
			
			
			

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()


	

if __name__ == '__main__':
    main()