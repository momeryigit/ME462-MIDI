from midibot_py.midibot import DifferentialDriveRobot as Robot
import time

# Constants
Kp = 1.0  # Proportional gain
Ki = 0.1  # Integral gain
Kd = 0.01  # Derivative gain

min_front = 30  # Minimum distance from front wall in cm
d_desired = 55  # Desired distance from the wall in cm
max_speed = 700  # Maximum wheel speed

# Initialize variables
previous_error = 0
integral = 0


def get_wheel_data(sensorId, sensorValue):
    d_left, d_front = None, None

    while not d_left and not d_front:
        if sensorId == "1":
            d_left = max(0, sensorValue)
        elif sensorId == "2":
            d_front = sensorValue

    calculate_wheel_speeds(d_left, d_front)


def calculate_wheel_speeds(d_left, d_front):
    global previous_error, integral

    if d_front < min_front:
        robot.stop()
        robot.send_command("s l -700")
        robot.send_command("s r 700")
        time.sleep(3.5)

        robot.stop()
        previous_error = 0
        integral = 0
        return

    # Calculate the error
    error = d_desired - d_left

    # Calculate the integral and derivative
    integral += error
    derivative = error - previous_error

    # Calculate control output
    u = Kp * error + Ki * integral + Kd * derivative

    # Update previous error
    previous_error = error

    # Adjust wheel speeds
    left_wheel_speed = max_speed - u
    right_wheel_speed = max_speed + u

    # Ensure the wheel speeds are within bounds
    left_wheel_speed = max(0, min(max_speed, left_wheel_speed))
    right_wheel_speed = max(0, min(max_speed, right_wheel_speed))

    robot.send_command(f"s l {int(left_wheel_speed)}")
    robot.send_command(f"s r {int(right_wheel_speed)}")


robot = Robot("COM6")
robot.connect(connection_type="serial")
robot.send_command("s l 1500")
robot.stop()
robot.send_command("s r 0")
robot.set_data_callback(get_wheel_data)
while True:
    continue