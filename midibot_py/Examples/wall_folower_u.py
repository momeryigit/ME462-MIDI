from midibot_py import DifferentialDriveRobot as Robot
import math
from time import sleep

u_sonic_dict = None

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
        if control_effort < -130:
            self.wall_tracking(control_effort)
        else:
            self.turning_phase(control_effort)

    def wall_tracking(self, control_effort):
        self._counter = 0
        self.robot.set_speed(700 + control_effort, 700 - control_effort)
    def turning_phase(self, control_effort):
        self._counter = 0
        self.robot.set_speed(600 + 0.2*control_effort, 700 - 1.87*control_effort)

robot = Robot("COM7", imu_connected=True)
pid = PIDController(8, 0.011, 0.02, 50.0, robot)  # Example PID parameters, adjust as needed


try:
    robot.connect(connection_type="serial")
    print("Robot connected")
except Exception as e:
    print("Robot connection error:", e)


# Main loop to check d3 and stop the robot if necessary
try:
    while robot.serial_running:
        u_sonic_dict = robot.sensors.u_sonic_data
        b_data = robot.sensors.b_switch_data
        imu_data = robot.sensors.imu_data
        print(u_sonic_dict)
        print(b_data)
        print(imu_data)
        # pid.calculate_control_effort(u_sonic_dict["u_3"])
        sleep(0.2)
except KeyboardInterrupt:
    robot.stop()
    robot.disconnect()