import time
import machine
from machine import freq
import gc

from comms import SerialComm
from utils import get_configs, read_configs, init_sensors_from_config, init_neopixels_from_config, init_steppers_from_config, init_heartbeat_from_config, command_handler, check_emergency

# Perform garbage collection
gc.collect()

def main():
    """
    Main function to initialize sensors and stepper motors, handle sensor polling, command handling, and emergency checks.
    """
    # Set the CPU frequency to 80 MHz
    freq(120000000)

    # Disable the heartbeat timer in the beginning Only way to do it on pico is to delete from memory.
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

    while True:
        try:
            try:
                # Get configuration data from the user
                get_configs(comm)
            except Exception as e:
                print(f"Error getting configurations: {str(e)}")
            
            # Read configurations from the config.json file
            config = read_configs()
            # Initialize sensors and stepper motors based on configuration
            sensors = init_sensors_from_config(config)
            neopixels = init_neopixels_from_config(config)
            steppers = init_steppers_from_config(config, neopixels)
            hb = init_heartbeat_from_config(config)

            # Start polling timers for all initialized sensors
            sensors.start_polling_timers()
            
            # Allow some time for initialization
            time.sleep(1)

            while True:
                # Main loop of the robot. Only breaks out of the loop if reinitialization is required.
                if hb:
                    # Send heartbeat signal
                    hb.beat()

                if not comm.pause_flag:
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
                    try:
                        command_handler(comm, msg, hb, steppers, sensors, neopixels)
                    except Exception as e:
                        if str(e) == "Reinitialize":
                            print("Reinitializing...")
                            break  # Break out of the loop to reinitialize
                        else:
                            print("error: ", e)

                # Perform garbage collection
                gc.collect()

        except Exception as e:
            print(f"Exception in main loop: {str(e)}")

if __name__ == "__main__":
    main()

