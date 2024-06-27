"""
This file contains utility functions for the Pico to handle peripheral configuration, commands, and emergency situations.
"""

import ujson
import time
from heartbeat import Heartbeat
from sensors import Sensors
from stepper import Steppers

def get_configs(comms):
    """
    Waits for a message from the user containing configuration data, then calls config_pico.

    Parameters:
    comms : SerialComm instance.
    """
    handshake = False
    
    while not handshake:
        try:
            msg = comms.read_parse()
            if msg:
                if msg[0] == "HANDSHAKE":
                    print("Handshake received.")
                    handshake = True
                    break
            else:
                print("Waiting for handshake.")
                time.sleep(1)
        except Exception as e:
            print(e)
            time.sleep(1)  # Avoid tight loop if there's an issue

    config_file = False
    while not config_file:
        try:
            msg = comms.read_message()
            if msg:
                print(msg)
                config_pico(msg)
                break
            else:
                print("Waiting for configuration file.")
        except Exception as e:
            print("Error while receiving configurations. Trying again soon:", e)
            time.sleep(1)

def config_pico(msg):
    """
    Check if the received messae is in json format then
    Save the received config.json message directly as config.json on the Pico.

    Parameters:
    msg : str
        The JSON configuration message.
    """
    try:
        # Check if the received message is in JSON format
        json_msg = ujson.loads(msg)

        # Write the received JSON message directly to config.json
        with open('config.json', 'w') as file:
            file.write(ujson.dumps(json_msg))

        print("Received config.json saved as config.json on the Pico.")
    
    except ValueError as e:
        print("Invalid JSON format:", e)

    except Exception as e:
        print("Error saving received config.json:", e)

def read_configs():
    """
    Read the config.json file and return the contents as a python dictionary.

    Returns:
    dict: The configuration data.
    """
    with open('config.json', 'r') as file:
        config_str = file.read()
    return ujson.loads(config_str)

def init_sensors_from_config(config):
    """
    Initialize sensors based on configuration.

    Parameters:
    config : dict
        The configuration data.

    Returns:
    Sensors: An instance of the Sensors class.
    """
    if "SENSORS" in config:
        sensors = Sensors()

        # Initialize ultrasonic sensors based on configuration
        if "ultrasonic" in config["SENSORS"]:
            try:
                sensors.types["ultrasonic"]["poll_rate"] = config["SENSORS"]["ultrasonic"].get("polling_rate", 1)
                for us_config in config["SENSORS"]["ultrasonic"]["sensors"]:
                    if us_config["enabled"]:
                        trigger_pin = us_config["trigger_pin"]
                        echo_pin = us_config["echo_pin"]
                        sensors.create_ultrasonic(us_config["id"], trigger_pin, echo_pin)
                        print(f"Ultrasonic sensor {us_config['id']} initialized on trigger pin {trigger_pin} and echo pin {echo_pin}")
            except Exception as e:
                print("Error initializing ultrasonic sensors:", e)

        # Initialize IMU sensor based on configuration
        if "imu" in config["SENSORS"]:
            try:
                if config["SENSORS"]["imu"]["enabled"]:
                    sensors.types["imu"]["poll_rate"] = config["SENSORS"]["imu"].get("polling_rate", 10)
                    sda_pin = config["SENSORS"]["imu"]["sda_pin"]
                    scl_pin = config["SENSORS"]["imu"]["scl_pin"]
                    sensors.create_imu(config["SENSORS"]["imu"]["id"], sda_pin, scl_pin)
                    print(f"IMU sensor {config['SENSORS']['imu']['id']} initialized on SDA pin {sda_pin} and SCL pin {scl_pin}")
            except Exception as e:
                print("Error initializing IMU sensor:", e)
        
        # Initialize bumper switches based on configuration
        if "bumper_switches" in config["SENSORS"]:
            try:
                sensors.types["bumper"]["poll_rate"] = config["SENSORS"]["bumper_switches"].get("polling_rate", 1)
                for bs_config in config["SENSORS"]["bumper_switches"]["sensors"]:
                    if bs_config["enabled"]:
                        pin = bs_config["pin"]
                        sensors.create_bumper(bs_config["id"], pin)
                        print(f"Bumper switch {bs_config['id']} initialized on pin {pin}")
            except Exception as e:
                print("Error initializing bumper switches:", e)
        
        # Set default emergency behavior based on configuration
        if "default_emergency_behavior" in config:
            sensors.default_emergency_behavior = config["default_emergency_behavior"]
    else:
        print("No sensors found in config.json")
        sensors = None
    return sensors

def init_steppers_from_json(config):
    """
    Initialize stepper motors based on configuration.

    Parameters:
    config : dict
        The configuration data.

    Returns:
    Steppers: An instance of the Steppers class after initializing the stepper motors.
    """
    if "stepper_motors" in config:
        steppers = Steppers()
        try:
            for motor_config in config["stepper_motors"]:
                if motor_config["enabled"]:
                    enable_pin = motor_config["enable_pin"]
                    step_pin = motor_config["step_pin"]
                    dir_pin = motor_config["dir_pin"]
                    led_pin = motor_config["led_pin"]
                    acc_step_size = motor_config.get("acc_step_size", 50)
                    acc_timer_period = motor_config.get("acc_timer_period", 10)
                    steppers.add_stepper(motor_config["id"], enable_pin, step_pin, dir_pin, led_pin, acc_step_size, acc_timer_period)
                    print(f"Stepper motor {motor_config['id']} initialized with enable pin {enable_pin}, step pin {step_pin}, dir pin {dir_pin}, led pin {led_pin}")
        except Exception as e:
            print("Error initializing stepper motors:", e)
    else:
        print("No stepper motors found in config.json")
        steppers = None
    return steppers

def init_heartbeat_from_json(config):
    """
    Initialize heartbeat based on configuration.

    Parameters:
    config : dict
        The configuration data.

    Returns:
    Heartbeat: An instance of the Heartbeat class.
    """
    if "heartbeat" in config:
        if config["heartbeat"]["enabled"]:
            heartbeat_timer = config["heartbeat"]["timer"]
            hb = Heartbeat(heartbeat_timer)
        else:
            hb = None
    else:
        print("No heartbeat found in config.json")
        hb = None
    return hb

def command_handler(msg, steppers, hb):
    """
    Handles commands received via serial communication.

    Parameters:
    msg : list
        The received command message.
    steppers : Steppers
        An instance of the Steppers class.
    hb : Heartbeat
        An instance of the Heartbeat class.
    """
    if msg is not None:
        mystring = ""
        for data in msg:
            mystring += str(data) + " "
        if msg[0] == "s":
            if msg[1] == "r":
                stepper = steppers.stepper_r
            elif msg[1] == "l":
                stepper = steppers.stepper_l
            if stepper:
                if int(float(msg[2])) == 0:
                    stepper.stop()
                else:
                    stepper.step(stepper.freq)
                    stepper.accelerate(int(float(msg[2])))
        elif msg[0] == "t":
            if msg[1] == "r":
                stepper = steppers.stepper_r
            elif msg[1] == "l":
                stepper = steppers.stepper_l
            if stepper:
                if int(float(msg[2])) or int(float(msg[3]))== 0:
                    stepper.stop()
                else:
                    stepper.tick(int(float(msg[2])), int(float(msg[3])))
        elif hb and msg[0] == "h":
            hb.feed()

def check_emergency(sensors):
    """
    Handle emergency situations where bumper switch is pressed or front ultrasonic data is less than 10 cm.

    Parameters:
    sensors : Sensors
        An instance of the Sensors class.

    Returns:
    bool: True if an emergency is detected, False otherwise.
    """
    if sensors.types["ultrasonic"]["sensor"][1]:
        emergency_ultrasonic = sensors.types["ultrasonic"]["sensor"][1].distance_cm()
        if emergency_ultrasonic < 10.0:
            return True
    return False
