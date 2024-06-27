from romer_midibot import DifferentialDriveRobot as Robot
import math
from time import sleep

u_sonic_dict = None

class PIDController:
    """
    PID Controller for controlling the movement of a differential drive robot based on distance error.
    """

    def __init__(self, Kp, Ki, Kd, desired_distance, robot):
        """
        Initialize PID Controller with coefficients and desired distance.

        Args:
            Kp (float): Proportional gain.
            Ki (float): Integral gain.
            Kd (float): Derivative gain.
            desired_distance (float): Target distance from obstacle.
            robot (DifferentialDriveRobot): Instance of the robot class for controlling movement.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.desired_distance = desired_distance
        self.previous_error = 0.0
        self.integral = 0.0
        self.robot = robot

    def calculate_control_effort(self, current_distance):
        """
        Calculate control effort (motor speeds) based on current distance error.

        Args:
            current_distance (float): Current distance measured by ultrasonic sensor.
        """
        error = self.desired_distance - current_distance
        self.integral += error
        derivative = error - self.previous_error

        # Calculate control effort based on PID formula
        control_effort = math.floor((self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)) if abs(error) > 0.1 else 0
        print("Control Effort: ", control_effort)

        # Adjust behavior based on control effort threshold
        if control_effort < -130:
            self.wall_tracking(control_effort)
        else:
            self.turning_phase(control_effort)

        self.previous_error = error

    def wall_tracking(self, control_effort):
        """
        Perform wall tracking behavior based on control effort.

        Args:
            control_effort (float): Control effort value influencing motor speeds.
        """
        self.robot.set_speed(700 + control_effort, 700 - control_effort)

    def turning_phase(self, control_effort):
        """
        Perform turning behavior based on control effort.

        Args:
            control_effort (float): Control effort value influencing motor speeds.
        """
        self.robot.set_speed(600 + 0.2 * control_effort, 700 - 1.87 * control_effort)


# Instantiate robot and PID controller
robot = Robot("COM7", imu_connected=True)
pid = PIDController(8, 0.011, 0.02, 50.0, robot)  # Example PID parameters, adjust as needed

try:
    # Connect to the robot
    robot.connect(connection_type="serial")
    print("Robot connected")
except Exception as e:
    print("Robot connection error:", e)

# Main loop to check distance and control the robot
try:
    while robot.serial_running:
        # Get ultrasonic sensor data
        u_sonic_dict = robot.get_sensor_data("u")
        # Calculate control effort based on distance error
        pid.calculate_control_effort(u_sonic_dict["u_3"])
        sleep(0.3)
except KeyboardInterrupt:
    # Stop the robot and disconnect on keyboard interrupt
    robot.stop()
    robot.disconnect()

