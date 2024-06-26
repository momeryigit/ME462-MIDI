import time
import machine
from machine import Timer, freq
import gc

from comms import SerialComm
from utils import get_configs, read_configs, init_sensors_from_config, init_steppers_from_json, init_heartbeat_from_json, command_handler, check_emergency

# Perform garbage collection
gc.collect()

def main():
    """
    Main function to initialize sensors and stepper motors, handle sensor polling, command handling, and emergency checks.
    """
    # Set the CPU frequency to 80 MHz
    freq(80000000)

    # Disable the heartbeat timer in the beginning
    machine.mem32[0x40058000] = machine.mem32[0x40058000] & ~(1 << 30)
    
    # Initialize serial communication
    comm = None    
    for _ in range(3):
        try:
            comm = SerialComm()
            print("Serial connection established")
            break
        except Exception as e:
            print(f"Failed to initialize serial port: {str(e)}")
            time.sleep(1)
    else:
        raise Exception("Failed to initialize serial communication after 3 attempts")
    
    try:
        # Get configuration data from the user
        get_configs(comm)
    except Exception as e:
        print(f"Error in main initialization: {str(e)}")
    
    # Read configurations from the config.json file
    config = read_configs()
    
    # Initialize sensors and stepper motors based on configuration
    sensors = init_sensors_from_config(config)
    steppers = init_steppers_from_json(config)
    hb = init_heartbeat_from_json(config)
    
    # Start polling timers for all initialized sensors
    sensors.start_polling_timers()

    # Allow some time for initialization
    time.sleep(1)

    while True:
        if hb:
            # Send heartbeat signal
            hb.beat()
        
        # Send sensory data if polling flags are set
        sensors.send_sensory_data()
        
        if sensors.default_emergency_behavior:
            # Check for emergency situations
            check_emergency(sensors)
        
        try:
            # Read and parse messages from serial communication
            msg = comm.read_parse()
        except Exception as e:
            print(type(e))
        
        if msg:
            # Handle received commands
            command_handler(msg, steppers, hb)
        
        # Perform garbage collection
        gc.collect()

if __name__ == "__main__":
    main()
