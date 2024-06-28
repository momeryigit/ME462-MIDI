from romer_midibot import DifferentialDriveRobot as Robot
import time


robot = Robot(serial_port= "COM6", imu_connected=False)
robot.connect(connection_type="serial")

try:
    while robot.serial_running:
        print("u", robot.get_sensor_data(sensor_type="u"))
        print("b", robot.get_sensor_data(sensor_type="b"))
        time.sleep(1)
except KeyboardInterrupt:
    robot.stop()
    robot.disconnect()