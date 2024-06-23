import time
import comms
import machine
from utils import get_configs

# Establish a connection via serial or socket
def pico_connection():
    """
    Tries to initialize the serial connection up to 3 times.
    """
    global comms
    for _ in range(3):
        try:
            comms.init_serial()
            break
        except Exception as e:
            print(f"Failed to initialize serial port: {str(e)}")
        time.sleep(1)
    else:
        raise Exception("Failed to initialize serial communication after 3 attempts")

def main():
    """
    Main function to establish a connection and get configurations.
    """
    machine.mem32[0x40058000] = machine.mem32[0x40058000] & ~(1<<30)
    try:
        pico_connection()
        get_configs(comms.serial_comms)
    except Exception as e:
        print(f"Error in main initialization: {str(e)}")

if __name__ == "__main__":
    main()
