from midibot_py import DifferentialDriveRobot as Robot
from time import sleep

# Change the serial port to the one you are using
serial_port = "COM7"

# Create a robot object to be initialized
robot = Robot(serial_port= serial_port, imu_connected=True)

# Try connecting to robot
try:
    robot.connect(connection_type="serial")
    print(robot.serial_comm.is_connected())
except Exception as e:
    print("Robot connection error:", e)

# Main loop to print sensor data
try:
    while robot.serial_running:
        u_sonic_dict = robot.sensors.u_sonic_data
        b_data = robot.sensors.b_switch_data
        imu_data = robot.sensors.imu_data
        print(u_sonic_dict)
        print(b_data)
        print(imu_data)
        sleep(0.2)
except KeyboardInterrupt:
    robot.stop()
    robot.disconnect()
