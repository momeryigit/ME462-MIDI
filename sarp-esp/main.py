import time
from machine import Timer, freq
import gc

from heartbeat import Heartbeat
import comms
import sensors
from utils import command_handler, check_emergency, set_u_sonic_flag, set_imu_flag, init_from_json

gc.collect()

comm = comms.serial_comms        # Retrieves the comm object from comms.py
timer_for_ultrasonic = Timer()   # Timer object for ultrasonic sensor
timer_for_imu = Timer()          # Timer object for IMU sensor

def main():
    """
    Main function to initialize sensors and stepper motors, and handle sensor polling, command handling, and emergency checks.
    """
    global comm

    sensors.init_sensors()  # Initialize sensors

    time.sleep(1)
    freq(80000000)
    
    init_from_json()
    sensors_obj = sensors.sensors_obj

    hb = Heartbeat(8000)

    # Timer-based interrupt to poll ultrasonic sensors if they are initialized
    if sensors_obj.types["ultrasonic"]["sensor"]:
        timer_for_ultrasonic.init(freq=1, mode=Timer.PERIODIC, callback=lambda t: set_u_sonic_flag())
    # Timer-based interrupt to poll IMU sensor if it is initialized
    if sensors_obj.types["imu"]["sensor"]:
        timer_for_imu.init(freq=5, mode=Timer.PERIODIC, callback=lambda t: set_imu_flag())

    while True:
        hb.beat()
        sensors_obj.send_sensory_data()
        check_emergency()
        try:
            msg = comm.read_parse()
        except Exception as e:
            print(type(e))
        if msg:
            command_handler(msg, hb)
        gc.collect()

if __name__ == "__main__":
    main()
