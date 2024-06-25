import time
import machine
from machine import Timer, freq
import gc

from heartbeat import Heartbeat
from comms import SerialComm
from sensors import Sensors
from stepper import Steppers
from utils import get_configs, command_handler, check_emergency, set_u_sonic_flag, set_imu_flag, init_from_json

gc.collect()

timer_for_ultrasonic = Timer()   # Timer object for ultrasonic sensor
timer_for_imu = Timer()          # Timer object for IMU sensor


def main():
    """
    Main function to initialize sensors and stepper motors, and handle sensor polling, command handling, and emergency checks.
    """
    machine.mem32[0x40058000] = machine.mem32[0x40058000] & ~(1<<30)
    comm = None    
    for _ in range(3):
        try:
            comm = SerialComm()
            print("Serial connnection established")
            break
        except Exception as e:
            print(f"Failed to initialize serial port: {str(e)}")
        time.sleep(1)
    else:
        raise Exception("Failed to initialize serial communication after 3 attempts")
    
    try:
        get_configs(comm)
    except Exception as e:
        print(f"Error in main initialization: {str(e)}")

    sensors = Sensors()  # Initialize sensors
    steppers = Steppers()

    time.sleep(1)
    freq(80000000)
    
    # Intializes sensors and steppers depending on config.json.
    init_from_json(sensors, steppers) 

#     hb = Heartbeat(8000)

    # Timer-based interrupt to poll ultrasonic sensors if they are initialized
    if sensors.types["ultrasonic"]["sensor"]:
        timer_for_ultrasonic.init(freq=5, mode=Timer.PERIODIC, callback=lambda t: set_u_sonic_flag(sensors))
    # Timer-based interrupt to poll IMU sensor if it is initialized
    if sensors.types["imu"]["sensor"]:
        timer_for_imu.init(freq=5, mode=Timer.PERIODIC, callback=lambda t: set_imu_flag(sensors))

    while True:
#         hb.beat()
        sensors.send_sensory_data()
        check_emergency(sensors)
        try:
            msg = comm.read_parse()
        except Exception as e:
            print(type(e))
        if msg:
            command_handler(msg, steppers)
        gc.collect()

if __name__ == "__main__":
    main()

