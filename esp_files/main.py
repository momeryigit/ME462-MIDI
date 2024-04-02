import time
import builtins
from machine import Pin, PWM, Timer
from stepper import Stepper
from serial import SerialComm
from socketclass import WifiServer

print("J")
beats = 0
def heartbeat(comms):
    global beats
    beats += 1
    comms.queue_mes('h ' + str(beats))
    
def toggle(mypin):
    mypin.value(not mypin.value())


def main():
    s1 = Stepper(26,16,steps_per_rev=600,speed_sps=10)
    s1.free_run(1)
    comms = None
    for i in range(3):
        try:
            # Set up Wi-Fi connection
            ssid = 'sarphotspot'
            password = 'lokomotif'
            comms = WifiServer(ssid, password, server_ip='0.0.0.0', server_port=8082)
            comms.connect_to_wifi()
            comms.start_server()
            comms.accept_connection()
            break
        except Exception as e:
            print(f'Failed to initialize socket port. {str(e)}')
            pass
    else:
        #print('Failed to initialize socket port after 3 attempts.')
        for i in range(3):
            try:
                comms = SerialComm()
                break
            except Exception as e:
                pass
                #print(f'Failed to initialize serial port. {str(e)}')
        else:
            #print('Failed to initialize serial port after 3 attempts.')
            return
    tim = Timer(0)
    tim2 = Timer(1)
    
    tim.init(freq=0.5, mode=Timer.PERIODIC, callback=lambda t: heartbeat(comms))
    led = Pin(20, Pin.OUT)
    tim2.init(freq=1, mode=Timer.PERIODIC, callback=lambda t: toggle(led))
    s1.enable(0)

    while True:
        msg = comms.read_parse()
        if msg != None:
            comms.send_message(str(msg))
            if msg[0] == 'sf':
                s1.enable(1)
                s1.free_run(0)
                s1.speed(int(msg[1]))
                s1.free_run(int(msg[2]))
                comms.send_message(str(s1.steps_per_sec))
            if msg[0] == 'ss':
                s1.free_run(0)
                s1.enable(1)
                s1.speed(int(msg[1]))
                s1.target(int(msg[2]))
                s1.overwrite_pos(0)
            if msg[0]=='q':
                s1.free_run(0)
                s1.enable(0)
                break
        comms.send_queue()
            
    
if __name__ == '__main__':
    main()