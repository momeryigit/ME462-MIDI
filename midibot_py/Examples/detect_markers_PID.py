#!/usr/bin/env py./detect_markes_PID.python
import cv2
import numpy as np
import os
import sys
import time
import math
import threading
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

# PID constants 
KP = 0.005  # Proportional gain
KI = 0.0000  # Integral gain
KD = 0.001 # Derivative gain

# Variables for PID control
prev_error = 0
integral = 0

# ArUco Dictionaries
ARUCO_DICT = {
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
}

def pid_control(error):
    global prev_error, integral
    
    # Proportional term
    proportional = KP * error
    
    # Integral term (with anti-windup)
    integral = integral + error
    
    # Derivative term
    derivative = KD * (error - prev_error)
    
    prev_error = error
    
    # Calculate angular velocity
    angular_vel = proportional + KI * integral + derivative
    
    return -1*angular_vel

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

def draw_markers_and_calculate_velocity(frame, corners, ids):
    """Draw markers on the frame and calculate velocity for robot movement."""
    if ids is not None:
        for marker_corner, marker_id in zip(corners, ids.flatten()):
            corners = marker_corner.reshape((4, 2))
            top_left, top_right, bottom_right, bottom_left = corners.astype(int)
            center_x, center_y = np.mean(corners, axis=0).astype(int)
            error = FRAME_WIDTH / 2 - center_x
            area = cv2.contourArea(corners)
            linear_vel = LINEAR_MAX - (area / 20000) * LINEAR_MAX
            angular_vel = pid_control(error)
            #draw_marker(frame, top_left, top_right, bottom_right, bottom_left, center_x, center_y, marker_id, error, area)
            return linear_vel, angular_vel, area
    return 0, 0

def draw_marker(frame, top_left, top_right, bottom_right, bottom_left, center_x, center_y, marker_id, error, area):
    """Draw the bounding box, center, error, and area of the ArUco marker."""
    cv2.polylines(frame, [np.array([top_left, top_right, bottom_right, bottom_left])], True, (0, 255, 0), 2)
    cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
    cv2.putText(frame, f"Error: {error}", (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, f"Area: {area}", (top_left[0], top_left[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, str(marker_id), (top_left[0], top_left[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

def capture_frames(cap, frame_queue, stop_event):
    """Capture frames from the camera and put them in the queue."""
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            continue
        frame_queue.append(frame)
        if len(frame_queue) > 1:
            frame_queue.pop(0)
def sign(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0
def process_frames(frame_queue, robot, aruco_dict, aruco_params, last_command_time, stop_event):
    """Process frames for ArUco markers and control the robot."""
    count = 0
    dropinterval = 10
    linear_vel = 0
    angular_vel = 0
    while not stop_event.is_set():
        if len(frame_queue) == 0:
            continue

        frame = frame_queue[0]
        corners, ids = detect_and_process_markers(frame, aruco_dict, aruco_params)
        if time.time() - last_command_time > COMMAND_INTERVAL:
            if ids is not None:
                count = 0
                linear_vel, angular_vel, area = draw_markers_and_calculate_velocity(frame, corners, ids)
                robot.linear_ang_speed_to_freq(linear_vel, angular_vel)
                print(f"Linear velocity: {linear_vel}, Angular velocity: {angular_vel}, Area: {area}")
            
            else:
                if count == 0:
                    drop_linear = abs(linear_vel) / dropinterval
                    drop_angular = abs(angular_vel) / dropinterval
                count +=1
                linear_vel = linear_vel - sign(linear_vel)*drop_linear
                angular_vel = angular_vel - sign(angular_vel) * drop_angular
                if count == dropinterval:
                    linear_vel = 0
                    angular_vel = 0
                robot.linear_ang_speed_to_freq(linear_vel, angular_vel)
                print(f"Linear velocity: {linear_vel}, Angular velocity: {angular_vel}")
            last_command_time = time.time()

        #cv2.imshow('frame', frame)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    break

def main():
    stop_event = threading.Event()
    try:

        print(f"[INFO] Detecting '{DESIRED_ARUCO_DICTIONARY}' markers...")
        aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[DESIRED_ARUCO_DICTIONARY])
        aruco_params = cv2.aruco.DetectorParameters_create()
        cap = setup_camera()

        robot = Robot(serial_port="/dev/ttyACM0")
        robot.connect("serial")

        frame_queue = []
        last_command_time = time.time()

        # Start frame capture thread
        frame_capture_thread = threading.Thread(target=capture_frames, args=(cap, frame_queue, stop_event))
        frame_capture_thread.daemon = True
        frame_capture_thread.start()

        # Start frame processing thread
        frame_processing_thread = threading.Thread(target=process_frames, args=(frame_queue, robot, aruco_dict, aruco_params, last_command_time, stop_event))
        frame_processing_thread.daemon = True
        frame_processing_thread.start()

        frame_capture_thread.join()
        frame_processing_thread.join()
        
    except KeyboardInterrupt:
        stop_event.set()
        robot.stop()
        print("\n[INFO] Program interrupted by user. Cleaning up...")
        frame_capture_thread.join()
        frame_processing_thread.join()
        cap.release()
        #cv2.destroyAllWindows()
        print("[INFO] Cleanup complete. Exiting.")

if __name__ == '__main__':
    main()
