import time
import builtins
from machine import Pin, PWM, Timer
from stepper import Stepper
from serial import SerialComm
from socketclass import WifiServer
from hcsr04 import HCSR04

beats = 0

def heartbeat(comms):
    global beats
    beats += 1
    comms.send_message('h ' + str(beats))

distance = 0
def getdist(ultrasonic, comms):
    global distance
    distance = ultrasonic.distance_cm()
    comms.send_message(distance)
def toggle(mypin):
    mypin.value(not mypin.value())


def main():
    s_r = Stepper(26, 16, steps_per_rev=600, speed_sps=10, invert_dir=True)
    s_l = Stepper(25, 27, steps_per_rev=600, speed_sps=10, timer_id=3) #step_pin,dir_pin,en_pin=None,steps_per_rev=600,speed_sps=10,invert_dir=False,timer_id=2
    ultrasonic = HCSR04(14, 17)
    en_pin = Pin(12, Pin.OUT)
    
    comms = None
    for i in range(1):
        try:
            # Set up Wi-Fi connection
            ssid = 'PuffPuffPeac'
            password = 'bilmem'
            comms = WifiServer(ssid, password, server_ip='0.0.0.0', server_port=8081)
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
    timer_for_heartbeat = Timer()
    timer_for_ultrasonic = Timer()
    
    timer_for_heartbeat.init(freq=0.5, mode=Timer.PERIODIC, callback=lambda t:heartbeat(comms))

    timer_for_ultrasonic.init(freq=1, mode=Timer.PERIODIC, callback =lambda t: getdist(ultrasonic, comms))
    

    en_pin.value(1)
    
    while True:
        try:
            msg = comms.read_parse()
        except Exception as e:
            print(e)
            dummy_timer = time.ticks_ms()/1000
            while True:
                if time.ticks_ms()/1000-dummy_timer >= 3:
                    break
        if msg != None:
            comms.send_message(str(msg))
            if distance >= 5.0:
                if msg[0] == 's':
                    if msg[1] == 'r':
                        s1 = s_r
                    elif msg[1] == 'l':
                        s1 = s_l
                    if int(float(msg[3])) == 0:
                        en_pin.value(1)
                        s1.free_run(0)
                        continue
                    en_pin.value(0)
                    s1.free_run(0)
                    s1.speed(float(msg[3]))
                    s1.free_run(int(float(msg[2])))
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
            else:
                s_r.speed(0)
                s_l.speed(0)
                en_pin.value(1)
        try:
            comms.send_queue()
        except Exception as e:
            print(e)
            pass
        
    en_pin.value(0)
if __name__ == '__main__':
    main()

