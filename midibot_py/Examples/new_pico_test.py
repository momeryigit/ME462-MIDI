from midibot_py import DifferentialDriveRobot as Robot
import time


robot = Robot(serial_port= "COM7", imu_connected=False, config_file=r"C:\Users\hp elitebook83\Desktop\ME462-MIDI\midibot_py\Examples\config.json")
robot.connect(connection_type="serial")

try:
    while robot.serial_running:
        print("u", robot.get_sensor_data(sensor_type="u"))
        print("b", robot.get_sensor_data(sensor_type="b"))
        time.sleep(1)
except KeyboardInterrupt:
    robot.stop()
    robot.disconnect()