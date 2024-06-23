from midibot_py import DifferentialDriveRobot as Robot
from time import sleep

serial_port = "/dev/ttyACM0"

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
        sleep(0.1)
        print('Running')
    print('Connnection lost!')
except KeyboardInterrupt:
    robot.stop()
    robot.disconnect()
    
