from midibot_py import DifferentialDriveRobot as Robot
import time
import json


robot = Robot(serial_port= "COM7", imu_connected=False)
robot.connect(connection_type="serial")

try:
    while robot.serial_running:
        print("u", robot.get_sensor_data(sensor_type="u"))
        print("b", robot.get_sensor_data(sensor_type="b"))
        time.sleep(1)
except KeyboardInterrupt:
    robot.stop()
    robot.disconnect()