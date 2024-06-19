import time
from serial_comm import SerialCommunication
from midibot import DifferentialDriveRobot
d1 = 1000.0
d2 = 1000.0
d3 = 0.0
d4 = 1000.0

class PIDController:
    def __init__(self, Kp, Ki, Kd, desired_distance):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.desired_distance = desired_distance
        self.previous_error = 0.0
        self.integral = 0.0

    def calculate_control_effort(self, current_distance):
        error = self.desired_distance - current_distance
        self.integral += error
        derivative = error - self.previous_error
        control_effort = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.previous_error = error
        return control_effort

    def adjust_robot_direction(self, control_effort):
        if control_effort > 0:
            robot.set_speed(400 - control_effort, 400 + control_effort)
        else:
            robot.set_speed(400 + control_effort, 400 - control_effort)

def managa_data(sensor_id, sensor_value):
    global d1, d2, d3, d4
    if sensor_id == "1":
        d1 = float(sensor_value)
    elif sensor_id == "2":
        d2 = float(sensor_value)
    elif sensor_id == "3":
        d3 = float(sensor_value)
    elif sensor_id == "4":
        d4 = float(sensor_value)
    else:
        return


# Threshold value for d3
d3_threshold = 50.0  # Example threshold value, adjust as needed

robot = DifferentialDriveRobot("/dev/ttyACM0")
robot.connect()
robot.set_data_callback(managa_data)  # Assuming this callback updates d3
robot.set_speed(2000, 2000)
robot_running = True

# Main loop to check d3 and stop the robot if necessary
try:
    while robot_running:
        print(f"d3: {d3}")
        if d3 > d3_threshold:  
            robot.stop()
            time.sleep(1)  
            robot.turn_90("left", 400)
            time.sleep(1)
            robot.set_speed(2000, 2000)

except KeyboardInterrupt:
    robot.stop()  # Ensure the robot stops if the script is interrupted
    robot.disconnect()