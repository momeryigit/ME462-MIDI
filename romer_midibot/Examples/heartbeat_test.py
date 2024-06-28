from romer_midibot import DifferentialDriveRobot as Robot
from time import sleep

serial_port = "/dev/ttyACM0"

# Create a robot object to be initialized
robot = Robot(serial_port= serial_port, imu_connected=False)

# Try connecting to robot
try:
    robot.connect(connection_type="serial")
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
    
