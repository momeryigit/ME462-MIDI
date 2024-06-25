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
    Save the received config.json message directly as config.json on the Pico.
    """
    try:
        # Write the received JSON message directly to config.json
        with open('config.json', 'w') as file:
            file.write(msg)

        print("Received config.json saved as config.json on the Pico.")

    except Exception as e:
        print("Error saving received config.json:", e)
        
def init_from_json(sensors, steppers):
    """
    Initializes sensors and steppers from a config.json file.
    """
    with open('config.json', 'r') as file:
        config_str = file.read()
    config = ujson.loads(config_str)

    # Initialize ultrasonic sensors based on configuration
    if "SENSORS" in config and "ultrasonic" in config["SENSORS"]:
        sensors.types["ultrasonic"]["poll_rate"] = config["SENSORS"]["ultrasonic"].get("polling_rate", 1)
        for us_config in config["SENSORS"]["ultrasonic"]["sensors"]:
            if us_config["enabled"]:
                trigger_pin = us_config["trigger_pin"]
                echo_pin = us_config["echo_pin"]
                sensors.create_ultrasonic(us_config["id"], trigger_pin, echo_pin)
                print(f"Ultrasonic sensor {us_config['id']} initialized on trigger pin {trigger_pin} and echo pin {echo_pin}")

    # Initialize bumper switches based on configuration
    if "SENSORS" in config and "bumper_switches" in config["SENSORS"]:
        sensors.types["bumper"]["poll_rate"] = config["SENSORS"]["bumper_switches"].get("polling_rate", 1)
        for bs_config in config["SENSORS"]["bumper_switches"]["sensors"]:
            if bs_config["enabled"]:
                pin = bs_config["pin"]
                sensors.create_bumper(bs_config["id"], pin)
                print(f"Bumper switch {bs_config['id']} initialized on pin {pin}")

    # Initialize IMU sensor based on configuration
    if "SENSORS" in config and "imu" in config["SENSORS"] and config["SENSORS"]["imu"]["enabled"]:
        sensors.types["imu"]["poll_rate"] = config["SENSORS"]["imu"].get("polling_rate", 10)
        sda_pin = config["SENSORS"]["imu"]["sda_pin"]
        scl_pin = config["SENSORS"]["imu"]["scl_pin"]
        sensors.create_imu(config["SENSORS"]["imu"]["id"], sda_pin, scl_pin)
        print(f"IMU sensor {config['SENSORS']['imu']['id']} initialized on SDA pin {sda_pin} and SCL pin {scl_pin}")

    # Initialize stepper motors based on configuration
    if "stepper_motors" in config:
        for motor_config in config["stepper_motors"]:
            if motor_config["enabled"]:
                enable_pin = motor_config["enable_pin"]
                step_pin = motor_config["step_pin"]
                dir_pin = motor_config["dir_pin"]
                led_pin = motor_config["led_pin"]
                steppers.add_stepper(motor_config["id"], enable_pin, step_pin, dir_pin, led_pin)
                print(f"Stepper motor {motor_config['id']} initialized with enable pin {enable_pin}, step pin {step_pin}, dir pin {dir_pin}, led pin {led_pin}")
                
    if "default_emergency_behavior" in config:
        sensors.default_emergency_behavior = config["default_emergency_behavior"]
# Handle commands received from user
def command_handler(msg, steppers):
    """
    Handles commands received via serial communication.
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
    emergency_ultrasonic = sensors.types["ultrasonic"]["sensor"][1].distance_cm()
    if  emergency_ultrasonic < 10.0:
        return True
