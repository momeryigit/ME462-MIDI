import time
from serial_comm import SerialCommunication
from midibot import DifferentialDriveRobot
import math

global d1, d2, d3, d4

class PIDController:
    def __init__(self, Kp, Ki, Kd, desired_distance, robot):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.desired_distance = desired_distance
        self.previous_error = 0.0
        self.integral = 0.0
        self._counter = 0
        self.robot = robot

    def calculate_control_effort(self, current_distance):

        error = self.desired_distance - current_distance
        self.integral += error
        derivative = error - self.previous_error
        control_effort = math.floor((self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)) if abs(error) > 0.1 else 0          
        print("Control Effort: ", control_effort)
        # if control_effort < -300:
        #     self.robot.stop()
        #     self.robot.turn_90("left", 900)
        #     self.robot.set_speed(1100, 1100)
        #     time.sleep(5)
        #     self.robot.stop()
        #     self.integral = 0
        #     self.previous_error = 0
        #     return
        self.previous_error = error
        if control_effort < -260:
            self.adjust_robot_turning(control_effort)
        else:
            self.adjust_robot_linear_direction(control_effort)

    def adjust_robot_linear_direction(self, control_effort):
        self._counter = 0
        self.robot.set_speed(1400 + control_effort, 1400 - control_effort)
    def adjust_robot_turning(self, control_effort):
        self._counter = 0
        self.robot.set_speed(1200 + 0.4*control_effort, 1400 - 2*1.87*control_effort)

robot = DifferentialDriveRobot("/dev/ttyACM0")
pid = PIDController(11, 3*0.011, 2*0.02, 50.0, robot)  # Example PID parameters, adjust as needed

def manage_data(data):
    global d3c
    d3 = data
    print("Data: ", d3)
    pid.calculate_control_effort(d3)
    # d4 = data[3]



robot.connect()
print("Robot connected")
robot.set_data_callback(manage_data)  # Assuming this callback updates d3
robot_running = True

# Main loop to check d3 and stop the robot if necessary
try:
    while robot_running:
        robot.serial_comm._parse_serial_data()

        continue
except KeyboardInterrupt:
    robot.stop()
    robot.disconnect()