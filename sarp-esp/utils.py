"""
This file contains utility functions for the Pico to handle peripheral_config, commands, and emergency situations.
"""
import ujson
import time

# Wait to get configuration from connection
def get_configs(comms):
    """
    Waits for a message from user containing configuration data then calls config_pico
    """
    handshake = False
    
    while not handshake:
        try:
            msg = comms.read_parse()
            if msg:
                if msg[0] == "HANDSHAKE":
                    print("handshake")
                    handshake = True
                    break
            else:
                print("Waiting for handshake")
                time.sleep(1)
        except Exception as e:
            print(e)
            time.sleep(1)  # Avoid tight loop if there's an issue
    config_file = False
    while not config_file:
#         print("in config")
        try:
            msg = comms.read_message()
            if msg:
                print(msg)
                config_pico(msg)
                break
            else:
                print("handshake")
        except Exception as e:
            
            print("Error while receiving configurations. Trying again soon:", e)
            time.sleep(1)

def config_pico(msg):
    """
    Update the config.json file on pico during boot. Called from boot.py
    """
    print("in second config")
    try:
        # Parse the received JSON data into a Python dictionary
        received_config = ujson.loads(msg)

        # Read the current config.json file into a string
        with open('config.json', 'r') as file:
            current_config_str = file.read()

        # Parse the current_config_str into a Python dictionary
        current_config = ujson.loads(current_config_str)

        # Update ultrasonic sensors
        for user_sensor in received_config['sensors']['ultrasonic']:
            for sensor in current_config['sensors']['ultrasonic']:
                if sensor['id'] == user_sensor['id']:
                    sensor['enabled'] = user_sensor['enabled']

        # Update bumper switches
        for user_sensor in received_config['sensors']['bumper_switches']:
            for sensor in current_config['sensors']['bumper_switches']:
                if sensor['id'] == user_sensor['id']:
                    sensor['enabled'] = user_sensor['enabled']

        # Update imu
        current_config['sensors']['imu']['enabled'] = received_config['sensors']['imu']['enabled']

        # Update stepper motors
        for user_motor in received_config['stepper_motors']:
            for motor in current_config['stepper_motors']:
                if motor['id'] == user_motor['id']:
                    motor['enabled'] = user_motor['enabled']

        # Serialize current_config dictionary to JSON format
        updated_config_str = ujson.dumps(current_config)

        # Write the updated configuration back to config.json
        with open('config.json', 'w') as file:
            file.write(updated_config_str)

        print("Updated config.json with received preferences.")

    except Exception as e:
        print("Error updating config.json:", e)
        

def init_from_json(sensors, steppers):
    """
    Initializes sensors and steppers from a config.json file.
    """
    with open('config.json', 'r') as file:
        config_str = file.read()
    config = ujson.loads(config_str)

    # Initialize ultrasonic sensors based on configuration
    for us_config in config["sensors"]["ultrasonic"]:
        if us_config["enabled"]:
            trigger_pin = us_config["trigger_pin"]
            echo_pin = us_config["echo_pin"]
            sensors.create_ultrasonic(us_config["id"], trigger_pin, echo_pin)
            print(f"Ultrasonic sensor {us_config['id']} initialized on trigger pin {us_config['trigger_pin']} and echo pin {us_config['echo_pin']}")

    # Initialize bumper switches based on configuration
    for bs_config in config["sensors"]["bumper_switches"]:
        if bs_config["enabled"]:
            pin = bs_config["pin"]
            sensors.create_bumper(bs_config["id"], pin)
            print(f"Bumper switch {bs_config['id']} initialized on pin {bs_config['pin']}")

    # Initialize IMU sensor based on configuration
    if config["sensors"]["imu"]["enabled"]:
        sda_pin = config["sensors"]["imu"]["sda_pin"]
        scl_pin = config["sensors"]["imu"]["scl_pin"]
        sensors.create_imu(config["sensors"]["imu"]["id"], sda_pin, scl_pin)
        print(f"IMU sensor {config['sensors']['imu']['id']} initialized on SDA pin {config['sensors']['imu']['sda_pin']} and SCL pin {config['sensors']['imu']['scl_pin']}")

    # Initialize stepper motors based on configuration
    for motor_config in config["stepper_motors"]:
        if motor_config["enabled"]:
            # if if is one, initialize the right motor, else initialize the left motor
            enable_pin = motor_config["enable_pin"]
            step_pin = motor_config["step_pin"]
            dir_pin = motor_config["dir_pin"]
            led_pin = motor_config["led_pin"]
            if motor_config["id"] == 1:
                steppers.add_stepper(motor_config["id"], enable_pin, step_pin, dir_pin, led_pin)
            else:
                steppers.add_stepper(motor_config["id"], enable_pin, step_pin, dir_pin, led_pin)
            print(f"Stepper motor {motor_config['id']} initialized with enable pin {motor_config['enable_pin']}, step pin {motor_config['step_pin']}, dir pin {motor_config['dir_pin']}, led pin {motor_config['led_pin']}")

# Handle commands received from user
def command_handler(msg, steppers):
    """
    Handles commands received via serial communication.
    """
    if msg is not None:
            mystring = ""
            for data in msg:
                mystring += str(data) + " "
            print(mystring)
            if msg[0] == "s":
                if msg[1] == "r":
                    stepper = steppers.stepper_r
                elif msg[1] == "l":
                    stepper = steppers.stepper_l
                if stepper:
                    if int(float(msg[2])) == 0:
                        stepper.stop()
                        print(stepper.pos)

                    else:
                        stepper.step(stepper.freq)
                        stepper.accelerate(int(float(msg[2])))
#             elif msg[0] == "h":
#                 hb.feed()

def set_u_sonic_flag(sensors):
    sensors.ultrasonic_flag = True

def set_imu_flag(sensors):
    sensors.IMU_flag = True

# Handle emergency situations where bumper switch is pressed or front ultrasonic data is less than 10 cm
def check_emergency(sensors):
    global s_r, s_l, sensors_obj
    emergency_ultrasonic = sensors.types["ultrasonic"]["sensor"][1].distance_cm()
    if  emergency_ultrasonic < 10.0:
        return True
