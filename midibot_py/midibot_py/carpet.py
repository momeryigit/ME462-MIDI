import time
from serial_comm import SerialCommunication
from midibot import DifferentialDriveRobot
import math


robot = DifferentialDriveRobot("/dev/ttyACM0")


robot.connect()
print("Robot connected")
robot.set_speed(3000, 3000)
time.sleep(5)
robot.stop()
robot.disconnect()

