from romer_midibot import DifferentialDriveRobot as Robot
from time import sleep

serial_port = "/dev/ttyACM0"

# Create a robot object to be initialized
robot = Robot(serial_port= serial_port, imu_connected=False)

def blink_leds_n_times(robot, n, bgr_tuple = (0, 255, 0)):
    for i in range(n):
        robot.fill_leds(bgr_tuple)
        sleep(0.1)
        robot.fill_leds((0, 0, 0))
        sleep(0.1)
    

# Try connecting to robot
try:
    robot.connect(connection_type="serial")
    sleep(2)
    blink_leds_n_times(robot, 3, (0, 255, 0))
except Exception as e:
    print("Robot connection error:", e)
    
# Main Loop
try:
    while robot.serial_running:
        # Sending robot twist messages.
        robot.send_twist(0.3, 0) # Move forward at 0.4 m/s
        sleep(3)
        robot.send_twist(-0.3, 0)
        sleep(3)
        robot.send_twist(0, 0.5) # Rotate right at 0.2 rad/s
        sleep(3)
        robot.send_twist(0, -0.5) # Rotate left at 0.2 rad/s
        sleep(3)
        robot.stop()
    
    
        blink_leds_n_times(robot, 3, (0, 255, 0))
        # Controlling robot with distance and time
        robot.set_distance_duration(2,2,5,5) # Make steppers rotate for 2 meters in 10 seconds
        sleep(12)
        robot.set_distance_duration(-2,-2,5,5) # Make steppers rotate for 2 meters in 10 seconds  
        sleep(12)
        blink_leds_n_times(robot, 3, (0, 255, 0))
        
        
        
except KeyboardInterrupt:
    robot.stop()
    robot.disconnect()
    